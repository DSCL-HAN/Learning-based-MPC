import gym
import numpy as np
from collections import defaultdict

class TwoLinkManipulatorEnv(gym.Env):
    def __init__(self, cfg):
        super().__init__()

        self.m1 = 5.6794
        self.m2 = 1.4730
        self.m3 = 1.7985

        self.dt = 0.05  
        self.cfg = cfg
        
        self.max_a = 5.0
        self.action_space = gym.spaces.Box(
            low=np.array([-self.max_a, -self.max_a]), 
            high=np.array([+self.max_a, self.max_a]),
            shape=(2,),
            dtype=np.float32
        )

        high = np.array([np.finfo(np.float32).max]*4, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)

        # episode length
        self.max_steps = cfg.episode_length
        self.steps = 0
        self.t = 0.0
        
        self.state = None
        self.z_parameter = None

    def reset(self):
        """
        Episode initialization: reset the state and return the observation
        """
        self.steps = 0
        self.t = 0.0
        
        x1 = 0.1
        x2 = -0.1
        x3 = 0.0
        x4 = 0.0
        z1 = 0.1
        z2 = 0
        self.state = np.array([x1, x2, x3, x4], dtype=np.float64)
        self.z_parameter = np.array([z1, z2], dtype=np.float64)
        return self._get_obs()

    def step(self, action):
        """
        1) action (scalar) in the range [-2, 2]
        2) Integrate using RK4/RK5 -> x(t + dt)
        3) Return reward, done, etc.
        """
                
        torque1 = float(action[0])*self.max_a 
        torque2 = float(action[1])*self.max_a 

        torque = np.array([torque1, torque2], dtype=np.float64)

        z_next = self._rkz5(self.z_parameter, self.t, self.dt)
        x_next = self._rkx5(self.state, self.z_parameter, torque, self.dt)
        self.state = x_next
        self.z_parameter = z_next
        self.steps += 1
        self.t += self.dt
        
        x1, x2, x3, x4 = x_next
        z1, z2 = z_next

        reward = - (x1**2 + x2**2 + x3**2 + x4**2)
        
        done = False

        if self.steps >= self.max_steps:
            done = True


        info = defaultdict(float)
        return self._get_obs(), float(reward), done, info

    def _get_obs(self):
        return self.state.astype(np.float32)

    def _plant_x(self, x, z, torque):
            """
            x = [x1, x2, x3, x4] => x1 = q1, x2 = q2, x3 = dq1, x4 = dq2
            torque = scalar -> first joint torque = torque, second joint torque = 0 (example)

            Dynamics:
            M = [[m1,               m2 * cos(x2 - x1)],
                [m2 * cos(x2 - x1),        m3       ]]
            C = [ -m2 * x4^2 * sin(x2 - x1),
                m2 * x3^2 * sin(x2 - x1)]
            ddq = M^-1 * ([torque, 0] - C)

            Return dxdt = [dx1, dx2, dx3, dx4]
                        = [x3,  x4,  ddq1, ddq2]
            """
            x1, x2, x3, x4 = x
            z1, z2 = z
            torque1, torque2 = torque
            # M
            M11 = self.m1
            M12 = self.m2 * np.cos(x2 - x1)
            M21 = M12
            M22 = self.m3
            Mmat = np.array([[M11, M12],
                            [M21, M22]], dtype=np.float64)

            # C
            C1 = - self.m2 * (x4**2) * np.sin(x2 - x1)
            C2 = + self.m2 * (x3**2) * np.sin(x2 - x1)
            Cvec = np.array([C1, C2], dtype=np.float64)
            

            tau_vec = np.array([torque1, torque2], dtype=np.float64)
            ext_vec = np.array([z1, z2], dtype=np.float64)

            invM = np.linalg.inv(Mmat)
            ddq = invM @ (tau_vec + ext_vec - Cvec)
            # ddq = invM @ (tau_vec - Cvec)
            ddq1, ddq2 = ddq[0], ddq[1]

            dxdt = np.array([
                x3,        
                x4,         
                ddq1,       
                ddq2        
            ], dtype=np.float64)
            
            return dxdt
    
    def _plant_z(self, z, t):
            z1, z2 = z
            ext_tau1 = z2

            W_1 = 1
            W_2 = 1
            W_3 = 1
            
            ext_tau1 = W_1*z2
            ext_tau2 = W_2*(W_3-z1**2)*z2 - z1

            ext_tau_vec = np.array([ext_tau1, ext_tau2], dtype=np.float64)

            
            dz = ext_tau_vec
            dz1, dz2 = dz[0], dz[1]

            dzdt = np.array([
                dz1,         
                dz2,         
            ], dtype=np.float64)
            
            return dzdt

    def _rkx5(self, x, z, u, dt):
        k1 = self._plant_x(x, z, u)*dt
        k2 = self._plant_x(x + 0.5*k1, z, u)*dt
        k3 = self._plant_x(x + 0.5*k2, z, u)*dt
        k4 = self._plant_x(x + k3,     z, u)*dt

        return x + ((k1+k4)/6+(k2+k3)/3)
    
    def _rkz5(self, z, t, dt):
        k1 = self._plant_z(z, t)*dt
        k2 = self._plant_z(z + 0.5*k1, t+ 0.5*dt)*dt
        k3 = self._plant_z(z + 0.5*k2, t + 0.5*dt)*dt
        k4 = self._plant_z(z + k3,     t + dt)*dt

        return z + ((k1+k4)/6+(k2+k3)/3)

