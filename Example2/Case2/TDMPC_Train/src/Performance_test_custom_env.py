import warnings
warnings.filterwarnings('ignore')
import os
os.environ['MKL_SERVICE_FORCE_INTEL'] = '1'
os.environ['MUJOCO_GL'] = 'egl'
import torch
import numpy as np
import gym
import time
import random
from pathlib import Path
import matplotlib.pyplot as plt

from cfg import parse_cfg
from env import make_env
from algorithm.tdmpc import TDMPC
import scipy.io as sio  

max_a = 5.0

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

def load_agent_and_env(cfg, model_path):
    env = make_env(cfg)
    agent = TDMPC(cfg)
    agent.load(model_path) 
    return env, agent

def run_mp_tracking(cfg, model_path, sim_time):
    
    set_seed(cfg.seed)
    
    # 1) Environment and agent creation
    env, agent = load_agent_and_env(cfg, model_path)
    sim_time = sim_time + env.dt
    env.max_steps = int(sim_time / env.dt)  
    
    # 2) Simulation initialization
    obs = env.reset()
    done = False
    
    # 3) Time/state recording
    time_list = [0.0]
    obs_list = [obs.copy()]
    action_list = []
 
    t_sec = 0.0
    
    while True:

        if t_sec >= sim_time:
            print(f"Reached {sim_time} seconds, stopping simulation.")
            break
        
        action = agent.plan(obs, eval_mode=True, step=0, t0=False)

        torque1 = float(action.cpu().numpy()[0])
        torque2 = float(action.cpu().numpy()[1])
        torque = np.array([torque1, torque2], dtype=np.float64)

        action_list.append(torque)  

        obs, reward, done, info = env.step(torque)

        t_sec += env.dt 
        time_list.append(t_sec)
        obs_list.append(obs.copy())
        
        if done:
            print(f"Environment done at t={t_sec:.3f}s. (steps={env.steps})")
            break

    obs_array = np.array(obs_list)  
    action_array = np.array(action_list)  #
    action_array = action_array*max_a 

    labels = ["q1", "q2", "q1d", "q2d"]
    
    plt.figure(figsize=(8,6))
    for i in range(obs_array.shape[1]):
        plt.plot(time_list, obs_array[:, i], label=labels[i])
    
    plt.title(f"MP-Custom (0~{sim_time} sec) with Learned Model")
    plt.xlabel("Time (s)")
    plt.ylabel("State value")
    plt.grid(True)
    plt.legend()
    
    plt.figure(figsize=(8,6))
    plt.plot(time_list[:-1], action_array[:,0], label="action(torque1)")
    plt.plot(time_list[:-1], action_array[:,1], label="action(torque2)")
    plt.title(f"MP-Custom (0~{sim_time} sec) - Action1")
    plt.xlabel("Time (s)")
    plt.ylabel("Control")
    plt.legend()
    
    plt.show()

    mdict = {
        "time_list": time_list,
        "obs_array": obs_array,
        "action_array": action_array
    }
    sio.savemat("Ex2_Case2_TDMPC_Performance_Test.mat", mdict)
    print("Saved 'Ex2_Case2_TDMPC_Performance_Test.mat' with time_list, obs_array, action_array.")
    
if __name__ == '__main__':

    cfg_path = Path().cwd() / 'cfgs'
    
    cfg = parse_cfg(cfg_path)
    
    cfg.task = 'mp-custom'
    print(cfg)
    
    model_path = './logs/mp-custom/state/default/1/models/Ex2_Case2_TDMPC_model.pt'  
    
    run_mp_tracking(cfg, model_path, 20.0)
    
   