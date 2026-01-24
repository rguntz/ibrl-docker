import copy
import math
import numpy as np
from functorch import combine_state_for_ensemble, vmap
import torch
import torch.nn as nn
import torch.nn.functional as F
from . import utils

from .networks import *
from . utils import optimal_transport_plan, cosine_distance, euclidean_distance

class VICEAgent():
	def __init__(self, *args,
				 discrim_hidden_size=128,
				 discrim_lr=3e-4,
				 mixup=True,
				 reward_type='logd',
				 share_encoder=False,
				 gaussian_noise_coef=0.,
				 use_trunk=True,
				 spectral_norm=True,
				 pos_dataset=None,
				 repr_dim = None,
				 **kwargs):

		self.discrim_hidden_size = discrim_hidden_size
		self.discrim_lr = discrim_lr
		self.discrim_hidden_size = discrim_hidden_size
		self.mixup = mixup
		self.reward_type = reward_type
		self.share_encoder = share_encoder
		self.gaussian_noise_coef = gaussian_noise_coef
		self.use_trunk = use_trunk
		self.spectral_norm = spectral_norm
		self.pos_dataset = pos_dataset
		self.num_goals = self.pos_dataset.shape[0]
		self.eps = 1e-10
		self.discrim_val_data = None

		self.obs_shape = kwargs['obs_shape']
		self.action_shape = kwargs['action_shape']
		self.lr = kwargs['lr']
		self.feature_dim = kwargs['feature_dim']
		self.hidden_dim = kwargs['hidden_dim']
		self.device = kwargs['device']
		self.reward_scale_factor = kwargs['reward_scale_factor']
		self.use_tb = kwargs['use_tb']

		# Changed log_std_bounds from [-10, 2] -> [-20, 2]
		self.log_std_bounds = [-20, 2]
		# Changed self.init_temperature to 1.0
		self.init_temperature = 1.0
		self.repr_dim = repr_dim

		# models
		self.encoder = Encoder(self.obs_shape).to(self.device)
		# overwrite hard-coded representation dim for convnet
		self.encoder.repr_dim = repr_dim if repr_dim else self.encoder.repr_dim
		self.model_repr_dim = self.encoder.repr_dim

		self.log_alpha = torch.tensor(np.log(self.init_temperature)).to(self.device)
		self.log_alpha.requires_grad = True
		# self.target_entropy = -self.action_shape[0] / 2.0
		self.log_alpha_optimizer = torch.optim.Adam([self.log_alpha], lr=self.lr)

		# optimizers
		self.encoder_opt = torch.optim.Adam(self.encoder.parameters(), lr=self.lr)
		# data augmentation
		self.aug = RandomShiftsAug(pad=4)
		self.encoder.train()

		self.discriminator = DiscrimVision(obs_shape=self.obs_shape,
											feature_dim=self.feature_dim,
											hidden_dim=self.discrim_hidden_size,
											repr_dim=self.model_repr_dim,
											create_inp_encoder=not bool(self.share_encoder),
											use_spectral_norm=self.spectral_norm,
											use_trunk=self.use_trunk,).to(self.device)
		
		self.discrim_opt = torch.optim.Adam(self.discriminator.parameters(), lr=self.discrim_lr)
		self.zero_alpha = torch.tensor(0.).to(self.device)

	def update_discriminator(self, neg_replay_iter):
		metrics = dict()

		batch_neg = next(neg_replay_iter)
		obs_neg = torch.from_numpy(batch_neg).float().to(self.device)

		# obs_neg = utils.to_torch(batch_neg, self.device)[0]
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

	def transition_tuple(self, replay_iter, demo_iter=None, oversample_count=None, online_buf_len=None, offline_buf_len=None):
		obs, action, reward, discount, next_obs, step_type, next_step_type = super().transition_tuple(replay_iter,
																									  demo_iter=demo_iter,
																									  oversample_count=oversample_count,
																									  online_buf_len=online_buf_len,
																									  offline_buf_len=offline_buf_len)
		with torch.no_grad():
			VICE_reward = self.compute_reward(next_obs, None)
		return (obs, action, VICE_reward, discount, next_obs, step_type, next_step_type)
