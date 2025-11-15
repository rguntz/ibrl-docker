import time

import numpy as np
import torch

import common_utils
from common_utils import ibrl_utils as utils
from env.trossen_wrapper import PixelTrossen


def run_eval(env_params, agent, num_game, seed, verbose=True) -> list[float]:
    """
    Run evaluation sequentially, one episode after another.
    
    Args:
        env_params: Dictionary of environment parameters
        agent: Agent with an act() method
        num_game: Number of episodes to run
        seed: Starting seed for evaluation
        verbose: Whether to print results
        
    Returns:
        List of success scores (floats) for each episode
    """
    env = PixelTrossen(**env_params)
    
    results = {}
    t = time.time()
    
    with torch.no_grad(), utils.eval_mode(agent):
        for i in range(num_game):
            episode_seed = seed + i
            np.random.seed(episode_seed)
            
            obs, _ = env.reset()
            success = False
            
            while not env.terminal:
                # Add batch dimension for agent
                batch_obs = {k: v.unsqueeze(0) for k, v in obs.items()}
                
                # Get action from agent
                batch_action = agent.act(batch_obs, eval_mode=True)
                action = batch_action[0]  # Remove batch dimension
                
                # Step environment
                obs, _, _, success, _ = env.step(action)
            
            results[episode_seed] = float(success)
    
    if verbose:
        print(f"total time {time.time() - t:.2f}")
        for episode_seed in sorted(list(results.keys())):
            print(f"seed {episode_seed}: score: {float(results[episode_seed])}")
        print(common_utils.wrap_ruler(""))
    
    # Return scores in order
    scores = []
    for episode_seed in sorted(list(results.keys())):
        scores.append(results[episode_seed])
    
    return scores