import copy
import math
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from . import utils

from . networks import *
from . utils import optimal_transport_plan, cosine_distance, euclidean_distance


class DiscriminatorRewardModel:
	def __init__(self, 
				 obs_shape,
				 device,
				 discrim_lr=3e-4,
				 discrim_hidden_size=128,
				 feature_dim=50,
				 mixup=True,
				 reward_type='logd',
				 share_encoder=False,
				 gaussian_noise_coef=0.,
				 use_trunk=True,
				 discrim_val_data=None,
				 pos_dataset=None,
				 state_dim=8,
				 ignore_view=None,
				 use_encoder=False,
				 encoder=None,
				 use_tb=False):
		
		self.obs_shape = obs_shape
		self.device = device
		self.discrim_lr = discrim_lr
		self.discrim_hidden_size = discrim_hidden_size
		self.feature_dim = feature_dim
		self.mixup = mixup
		self.eps = 1e-10
		self.reward_type = reward_type
		self.share_encoder = share_encoder
		self.gaussian_noise_coef = gaussian_noise_coef
		self.use_trunk = use_trunk
		self.discrim_val_data = discrim_val_data
		self.pos_dataset = pos_dataset
		self.use_encoder = use_encoder
		self.use_tb = use_tb

		# If using external encoder (e.g., from RL agent)
		if use_encoder and encoder is not None:
			self.encoder = encoder
			self.model_repr_dim = encoder.repr_dim
			self.encoder_opt = None
		else:
			# Create our own encoder
			self.encoder = Encoder(obs_shape).to(device)
			self.model_repr_dim = self.encoder.repr_dim
			self.encoder_opt = torch.optim.Adam(self.encoder.parameters(), lr=discrim_lr)

		# Create discriminator
		self.discriminator = DiscrimVisionFranka(obs_shape=self.obs_shape,
												 feature_dim=self.feature_dim,
												 hidden_dim=discrim_hidden_size,
												 repr_dim=self.model_repr_dim,
												 create_inp_encoder=not bool(self.share_encoder),
												 state_dim=state_dim,
												 ignore_view=ignore_view).to(self.device)

		# Preload goal states as torch tensors
		if pos_dataset is not None:
			obs_img_pos, obs_state_pos = self.pos_dataset['images'], self.pos_dataset['states']
			self.obs_img_pos, self.obs_state_pos = utils.to_torch((obs_img_pos, obs_state_pos), self.device)
			self.num_goals = self.obs_img_pos.shape[0]
		else:
			self.obs_img_pos = None
			self.obs_state_pos = None
			self.num_goals = 0

		self.discrim_opt = torch.optim.Adam(self.discriminator.parameters(), lr=discrim_lr)
		self.aug = RandomShiftsAug(pad=4)
		self.training = True
		self.discriminator.train()

	def train(self, training=True):
		self.training = training
		self.discriminator.train(training)
		if self.encoder_opt is not None:
			self.encoder.train(training)

	def update_discriminator(self, neg_replay_iter):
		metrics = dict()

		batch_neg = next(neg_replay_iter)
		obs_neg = utils.to_torch(batch_neg, self.device)[0]
		num_pos = num_neg = obs_neg.shape[0]

		# shuffle the goal states then sample a minibatch
		if self.num_goals < num_neg:
			ridxs = torch.cat([torch.randperm(self.num_goals) for _ in range(math.ceil(num_neg / self.num_goals))])[:num_neg]
		else:
			ridxs = torch.randperm(self.num_goals)[:num_neg]
		obs_pos = self.pos_dataset[ridxs].to(self.device).type(torch.cuda.FloatTensor)

		'''augment the images and encode + trunk images before mixup,
		the shared encoder has not seen mixed up images.'''
		obs_pos = self.aug(obs_pos.float())
		obs_neg = self.aug(obs_neg.float())

		# frozen shared encoder
		if self.share_encoder == 1:
			with torch.no_grad():
				obs_pos = self.encoder(obs_pos)
				obs_neg = self.encoder(obs_neg)
		# update shared encoder
		elif self.share_encoder == 2:
			obs_pos = self.encoder(obs_pos)
			obs_neg = self.encoder(obs_neg)
		# use and train discriminator's own encoder
		elif self.share_encoder == 0:
			obs_pos = self.discriminator.encode(obs_pos)
			obs_neg = self.discriminator.encode(obs_neg)

		if self.use_trunk:
			obs_pos = self.discriminator.trunk_pass(obs_pos)
			obs_neg = self.discriminator.trunk_pass(obs_neg)

		pos_input = obs_pos
		neg_input = obs_neg

		# exit()
		if self.mixup:
			disc_inputs = torch.cat((pos_input, neg_input), 0)
			labels = torch.cat((torch.ones(num_pos, 1), torch.zeros(num_neg, 1)), 0).to(self.device)

			# Beta(1,1) is Uniform[0,1]
			beta_dist = torch.distributions.beta.Beta(torch.tensor([1.0]), torch.tensor([1.0]))
			l = beta_dist.sample([num_pos + num_neg])
			mixup_coef = torch.reshape(l, (num_pos + num_neg, 1)).to(self.device)

			# permute the images and labels for mixing up
			ridxs = torch.randperm(num_pos + num_neg)
			perm_labels = labels[ridxs]
			perm_disc_inputs = disc_inputs[ridxs]

			# create mixed up inputs
			images = disc_inputs * mixup_coef + perm_disc_inputs * (1 - mixup_coef)
			labels = labels * mixup_coef + perm_labels * (1 - mixup_coef)
		else:
			images = torch.cat((pos_input, neg_input), 0)
			labels = torch.cat((torch.ones(num_pos, 1), torch.zeros(num_neg, 1)), 0).to(self.device)

		loss = torch.nn.BCELoss()
		m = nn.Sigmoid()

		images = images + self.gaussian_noise_coef * torch.randn_like(images)
		output = m(self.discriminator.final_out(images))
		discrim_loss = loss(output, labels)
		
		if self.share_encoder == 2:
			self.encoder_opt.zero_grad(set_to_none=True)

		self.discrim_opt.zero_grad(set_to_none=True)
		discrim_loss.backward()
		self.discrim_opt.step()

		if self.share_encoder == 2:
			self.encoder_opt.step()

		if self.use_tb:
			metrics['discriminator_loss'] = discrim_loss.item()
			if not self.mixup:
				metrics['discriminator_acc'] = ((output > 0.5) == labels).type(torch.float).mean().item()
			if self.discrim_val_data is not None:
				with torch.no_grad():
					_, output = self.compute_reward(self.discrim_val_data['observations'], return_sig=True, evald=True)
					val_labels = torch.ones(self.discrim_val_data['observations'].shape[0], 1).to(self.device)
					metrics['val_loss'] = loss(output, val_labels).item()
					metrics['val_acc'] = ((output > 0.5) == val_labels).type(torch.float).mean().item()

		return metrics

	def compute_reward(self, obs, action=None, return_sig=False, evald=False):
		del action
		if evald and type(obs) is np.ndarray:
			obs = torch.from_numpy(obs).to(self.device)

		if self.share_encoder:
			obs = self.encoder(obs)

		sig_term = torch.sigmoid(self.discriminator(obs))
		if self.reward_type == 'logd':
			actual_reward = torch.log(torch.minimum(sig_term + self.eps, torch.tensor(1.)))
		else:
			actual_reward = -torch.log(1 - sig_term + self.eps)

		if not return_sig:
			return actual_reward
		else:
			return actual_reward, sig_term

	def save(self, filepath):
		"""Save discriminator and encoder state"""
		state = {
			'discriminator': self.discriminator.state_dict(),
			'encoder': self.encoder.state_dict() if self.encoder_opt is not None else None,
		}
		torch.save(state, filepath)

	def load(self, filepath):
		"""Load discriminator and encoder state"""
		state = torch.load(filepath)
		self.discriminator.load_state_dict(state['discriminator'])
		if self.encoder_opt is not None and state['encoder'] is not None:
			self.encoder.load_state_dict(state['encoder'])
