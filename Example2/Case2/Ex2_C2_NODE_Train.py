import math
import numpy as np
from IPython.display import clear_output
from tqdm import tqdm_notebook as tqdm
import scipy.io as sio

import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns
sns.color_palette("bright")
import matplotlib as mpl
import matplotlib.cm as cm

import torch
from torch import Tensor
from torch import nn
from torch.nn  import functional as F 
from torch.autograd import Variable

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
data_type = torch.float32

#################################################
# 1) Forward/Backward/ODE Solver
#################################################
def ode_solve(z0, t0, t1, f):
    """
    Forward Step
    """
    h_max = 0.05
    n_steps = math.ceil((abs(t1 - t0)/h_max).max().item())
    h = (t1 - t0)/n_steps
    t = t0.to(device=device, dtype=data_type)
    z = z0.to(device=device, dtype=data_type)
    for _ in range(n_steps):
        fk1=f(z, t)*h
        fk2=f(z + 0.5*fk1, t + 0.5*h)*h
        fk3=f(z + 0.5*fk2, t + 0.5*h)*h
        fk4=f(z + fk3,     t + h)*h
        z = z + ((fk1+fk4)/6+(fk2+fk3)/3)
        t = t + h

    return z 

def ode_solve_b(z0, t0, t1, f):
    """
    Backward Step
    """
    h_max = 0.05
    n_steps = math.ceil((abs(t1 - t0)/h_max).max().item()) 
    h = (t1 - t0)/n_steps
    t = t0.to(device=device, dtype=data_type)
    z = z0.to(device=device, dtype=data_type)
    for _ in range(n_steps):
        k1=f(z, t)*h
        k2=f(z + 0.5*k1, t + 0.5*h)*h
        k3=f(z + 0.5*k2, t + 0.5*h)*h
        k4=f(z + k3,     t + h)*h
        z = z + ((k1+k4)/6+(k2+k3)/3)
        t = t + h

    return z 

def Spiral_Trajectory(z, t):
    W_1 = 1
    W_2 = 1
    W_3 = 1
    dz = torch.zeros_like(z)
    dz[:, 0] = W_1*z[:,1]
    dz[:, 1] = W_2*(W_3-z[:,0]**2)*z[:,1] - z[:,0]

    return dz

def rk4_step(z, t, h):
    k1 = Spiral_Trajectory(z, t)*h
    k2 = Spiral_Trajectory(z + 0.5*k1, t + 0.5*h)*h
    k3 = Spiral_Trajectory(z + 0.5*k2, t + 0.5*h)*h
    k4 = Spiral_Trajectory(z + k3,     t + h)*h

    return z + ((k1+k4)/6+(k2+k3)/3)

#################################################
# 2) Neural ODE Adjoint
#################################################
class ODEF(nn.Module):
    def forward_with_grad(self, z, t, grad_outputs):
        """Compute f and a df/dz, a df/dp, a df/dt"""
        batch_size = z.shape[0]
        out = self.forward(z, t)
        a = grad_outputs
        adfdz, adfdt, *adfdp = torch.autograd.grad((out,), (z, t) + tuple(self.parameters()), grad_outputs=(a),
            allow_unused=True, retain_graph=True
        )
        # grad method automatically sums gradients for batch items, we have to expand them back 
        if adfdp is not None:
            adfdp = torch.cat([p_grad.flatten() for p_grad in adfdp]).unsqueeze(0)
            adfdp = adfdp.expand(batch_size, -1) / batch_size
        if adfdt is not None:
            adfdt = adfdt.expand(batch_size, 1) / batch_size 
            # .expand makes the expanded one with batch_size by 1 dimension

        return out, adfdz, adfdt, adfdp

    def flatten_parameters(self):
        p_shapes = []
        flat_parameters = []
        for p in self.parameters():
            p_shapes.append(p.size())
            flat_parameters.append(p.flatten()) 

        return torch.cat(flat_parameters)
    
class ODEAdjoint(torch.autograd.Function):
    @staticmethod
    def forward(ctx, z0, t, flat_parameters, func):
        assert isinstance(func, ODEF)
 
        bs, *z_shape = z0.size() 
        time_len = t.size(0)       
        with torch.no_grad():
            z = torch.zeros(time_len, bs, *z_shape).to(z0).to(device=device, dtype=data_type)
            z[0] = z0
            for i_t in range(time_len - 1):
                z0 = ode_solve(z0, t[i_t], t[i_t+1], func)
                z[i_t+1] = z0
        ctx.func = func
        ctx.save_for_backward(t, z.clone(), flat_parameters)

        return z

    @staticmethod
    def backward(ctx, dLdz):
        """
        dLdz shape: time_len, batch_size, *z_shape
        """
        func = ctx.func
        t, z, flat_parameters = ctx.saved_tensors
        time_len, bs, *z_shape = z.size()
        n_dim = np.prod(z_shape)
        n_params = flat_parameters.size(0)

        # Dynamics of augmented system to be calculated backwards in time
        def augmented_dynamics(aug_z_i, t_i):
            z_i, a = aug_z_i[:, :n_dim], aug_z_i[:, n_dim:2*n_dim]            
            z_i = z_i.view(bs, *z_shape)
            a = a.view(bs, *z_shape)
            with torch.set_grad_enabled(True):
                t_i = t_i.detach().requires_grad_(True)
                z_i = z_i.detach().requires_grad_(True)
                func_eval, adfdz, adfdt, adfdp = func.forward_with_grad(z_i, t_i, grad_outputs=a)  
                adfdz = adfdz.to(z_i) if adfdz is not None else torch.zeros(bs, *z_shape).to(z_i)
                adfdp = adfdp.to(z_i) if adfdp is not None else torch.zeros(bs, n_params).to(z_i)
                adfdt = adfdt.to(z_i) if adfdt is not None else torch.zeros(bs, 1).to(z_i)

            func_eval = func_eval.view(bs, n_dim)
            adfdz = adfdz.view(bs, n_dim) 

            return torch.cat((func_eval, -adfdz, -adfdp, -adfdt), dim=1)
     
        dLdz = dLdz.view(time_len, bs, n_dim)
    
        with torch.no_grad():
            ## Create placeholders for output gradients
            # Prev computed backwards adjoints to be adjusted by direct gradients
            adj_z = torch.zeros(bs, n_dim).to(dLdz)
            adj_p = torch.zeros(bs, n_params).to(dLdz)
            # In contrast to z and p we need to return gradients for all times
            adj_t = torch.zeros(time_len, bs, 1).to(dLdz)

            for i_t in range(time_len-1, 0, -1):
                z_i = z[i_t]
                t_i = t[i_t]
                f_i = func(z_i, t_i).view(bs, n_dim)

                # Compute direct gradients
                dLdz_i = dLdz[i_t]
                dLdt_i = torch.bmm(torch.transpose(dLdz_i.unsqueeze(-1), 1, 2), f_i.unsqueeze(-1))[:, 0]

                # Adjusting adjoints with direct gradients
                adj_z += dLdz_i
                adj_t[i_t] = adj_t[i_t] - dLdt_i

                # Pack augmented variable
                aug_z = torch.cat((z_i.view(bs, n_dim), adj_z, torch.zeros(bs, n_params).to(z), adj_t[i_t]), dim=-1)

                # Solve augmented system backwards
                aug_ans = ode_solve_b(aug_z, t_i, t[i_t-1], augmented_dynamics)

                # Unpack solved backwards augmented system
                adj_z[:] = aug_ans[:, n_dim:2*n_dim]
                adj_p[:] += aug_ans[:, 2*n_dim:2*n_dim + n_params]
                adj_t[i_t-1] = aug_ans[:, 2*n_dim + n_params:]

                del aug_z, aug_ans

            ## Adjust 0 time adjoint with direct gradients
            # Compute direct gradients 
            dLdz_0 = dLdz[0]
            dLdt_0 = torch.bmm(torch.transpose(dLdz_0.unsqueeze(-1), 1, 2), f_i.unsqueeze(-1))[:, 0]

            # Adjust adjoints
            adj_z += dLdz_0
            adj_t[0] = adj_t[0] - dLdt_0

        return adj_z.view(bs, *z_shape), adj_t, adj_p, None
    
class NeuralODE(nn.Module):
    def __init__(self, func):
        super(NeuralODE, self).__init__()
        assert isinstance(func, ODEF)
        self.func = func
        
    def forward(self, z0, t=Tensor([0., 1.]), return_whole_sequence=False):    
        t = t.to(z0) 
        t = t.to(device=device, dtype=data_type)
        z0 = z0.to(device=device, dtype=data_type)
        z = ODEAdjoint.apply(z0, t, self.func.flatten_parameters(), self.func)

        if return_whole_sequence:
            return z
        else:
            return z[-1]
        
#################################################
# 3) Neural ODE Structure
#################################################
class LinearODEF_true(ODEF):
    def __init__(self):
        super(LinearODEF_true, self).__init__()
        self.lin1 = nn.Linear(2, 2, bias=False)

    def forward(self, x, t):
        z_out = torch.zeros(1,2).to(device=device, dtype=data_type)
        W_1 = 1
        W_2 = 1
        W_3 = 1

        z_out[:, 0] = W_1*x[:,1]
        z_out[:, 1] = W_2*(W_3-x[:,0]**2)*x[:,1] - x[:,0]

        return z_out
    
class LinearODEF_train(ODEF):
    def __init__(self, hidden_dim=128):
        super(LinearODEF_train, self).__init__()
        
        self.net = nn.Sequential(
            nn.Linear(5, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 2)
        ).to(device=device, dtype=data_type)

    def forward(self, x, t):
        if t.dim() == 1:
            t = t.view(-1, 1)
        x1 = x[:, 0:1]  
        x2 = x[:, 1:2]  #
        x1_sq = x1**2
        x1x2  = x1 * x2
        inp = torch.cat([x1, x2, t, x1_sq, x1x2], dim=1) 
        z_out = self.net(inp)  

        return z_out
    
class SpiralFunctionExample(LinearODEF_true):
    def __init__(self):
        super(SpiralFunctionExample, self).__init__() 
class RandomLinearODEF(LinearODEF_train):
    def __init__(self):
        super(RandomLinearODEF, self).__init__() 

#################################################
# 4) Training Neural ODE
################################################# 
def conduct_experiment(ode_true, ode_trained, n_steps, name, plot_freq, Tf, n_points, z_initial, min_delta_time, max_delta_time, Nd):
    # Create data
    z0 = Variable(torch.Tensor([z_initial])).to(device=device, dtype=data_type) 
    index_np = np.arange(0, n_points, 1, dtype=int)
    index_np = np.hstack([index_np[:, None]])
    times_np = np.linspace(0, Tf, num=n_points)
    times_np = np.hstack([times_np[:, None]])
    times = torch.from_numpy(times_np[:, :, None]).to(z0).to(device=device, dtype=data_type) 
    
    times = times_torch = torch.tensor(times, device=device, dtype=data_type).view(-1,1,1)
    obs = ode_true(z0, times, return_whole_sequence=True).detach() 
    
    obs = obs + torch.randn_like(obs) * 0.0350
    obs_temp = obs
    optimizer = torch.optim.Adam(ode_trained.parameters(), lr=1e-3)

    # Get trajectory of random timespan 
    def create_batch():
        t0 = np.random.uniform(0, Tf - max_delta_time)
        t1 = t0 + np.random.uniform(min_delta_time, max_delta_time)
        idx = sorted(np.random.permutation(index_np[(times_np > t0) & (times_np < t1)])[:Nd])
        # 0 ~ Tf, the max number of idx is max_points_num (Nd)
        
        obs_ = obs[idx] 
        ts_ = times[idx]
        
        return obs_, ts_
    
    # Plot in real time
    plt.ion()
    fig, axes = plt.subplots(1,2, figsize=(12,4))
    ax_loss, ax_traj = axes
    loss_list_NODE = []

    (line_loss,) = ax_loss.plot([], [], 'r-', label='Loss')
    ax_loss.set_xlabel("Train Step")
    ax_loss.set_ylabel("MSE Loss")
    ax_loss.grid(True)
    ax_loss.legend()

    t_ = times_np[:,0]
    
    obs_np = obs[:,0,:].cpu().numpy()  
  
    (line_z1_true,) = ax_traj.plot(obs_np[:,0], obs_np[:,1], 'r-', label='True $z$')
    (line_z_pred,) = ax_traj.plot(obs_np[:,0]*0.0, obs_np[:,1]*0.0, 'm-', label='NODE $z$')


    ax_traj.set_xlabel("z_{1}")
    ax_traj.set_ylabel("z_{2}")
    ax_traj.grid(True)
    ax_traj.legend()

    for i in range(n_steps):
        loss_sum = 0.0
        for j in range(len(obs)-1):
            z0_j = obs[j]       
            t0_j = times[j]         
            t1_j = times[j+1]       

            z1_pred = ode_solve(z0_j, t0_j, t1_j, f = ode_trained.func) 
            z1_true = obs[j+1]   
            loss_sum += F.mse_loss(z1_pred, z1_true)

        loss = loss_sum / (len(obs)-1)
        loss_list_NODE.append(loss.item())
         
        optimizer.zero_grad() # Before backward, erase prior values of the grad
        loss.backward(retain_graph=True) 
        optimizer.step() # step() is to update weighting parameters

        if i % plot_freq == 0:
            print(f"[Step {i}] Loss = {loss.item():.6f}")

            line_loss.set_xdata(np.arange(len(loss_list_NODE)))
            line_loss.set_ydata(loss_list_NODE)
            ax_loss.relim()
            ax_loss.autoscale_view()

            with torch.no_grad():
                pred_full = ode_trained(z0, times, return_whole_sequence=True)
            pred_np = pred_full[:,0,:].cpu().numpy()  
            
            line_z_pred.set_xdata(pred_np[:,0])
            line_z_pred.set_ydata(pred_np[:,1])

            ax_traj.relim()
            ax_traj.autoscale_view()

            plt.draw()
            plt.pause(0.01)
    
    print("Training has been successfully completed. Closing this window will terminate the script.")
    plt.ioff()
    plt.show()
    
    obs_temp = obs  
    ts_temp = times
    Loss_NODE_out = np.array(loss_list_NODE)

    return obs_temp, ts_temp, Loss_NODE_out

"""
Simulation Parameters
"""
ode_true = NeuralODE(SpiralFunctionExample())
ode_trained = NeuralODE(RandomLinearODEF())

Tf = 20
plot_freq = 10
n_points = 200
h_ode = 0.05
z_initial = [0.1, 0.0]
n_steps = 500
min_delta_time = 1
max_delta_time = 10.0
Nd = 50
obs_temp_out, ts_temp_out, Loss_NODE_out = conduct_experiment(
                                            ode_true, ode_trained,
                                            n_steps = n_steps,
                                            name = "linear",
                                            plot_freq = plot_freq,
                                            Tf = Tf,           
                                            n_points = n_points,
                                            z_initial = z_initial,
                                            min_delta_time = min_delta_time,
                                            max_delta_time = max_delta_time,
                                            Nd = Nd)

## Trained Neural ODE Data Save
PATH = "./NODE_Ex2_Case2_model.pt"
# torch.save(ode_trained.state_dict(), PATH)
# state_dict = ode_trained.state_dict()
# data_dict = {
#     "W1": state_dict["func.net.0.weight"].cpu().numpy(),
#     "B1": state_dict["func.net.0.bias"].cpu().numpy(),
#     "W2": state_dict["func.net.2.weight"].cpu().numpy(),
#     "B2": state_dict["func.net.2.bias"].cpu().numpy(),
#     "W3": state_dict["func.net.4.weight"].cpu().numpy(),
#     "B3": state_dict["func.net.4.bias"].cpu().numpy(),
# }

# sio.savemat("NODE_Weight.mat", data_dict)

#################################################
# 5) Load & Prediction using Neural ODE
#################################################  
ckpt = torch.load(PATH)
ode_trained.load_state_dict(ckpt)
ode_trained.eval()

time_test = np.arange(0, Tf+h_ode, h_ode)
z_init = torch.tensor([z_initial], dtype=data_type, device=device) 
z_NODE_list = []
z_True_list = []
z_current = z_init.clone()
z_True_current = z_init.clone()
step_i = torch.tensor(0, device=device)

with torch.no_grad():
    for i in range(len(time_test)):
        if i == len(time_test)-1:
            z_NODE_list.append(z_current.clone())
            break
        t0 = time_test[i]
        t1 = time_test[i+1]

        t_tensor = torch.tensor([[ [t0], [t1] ]],
                                dtype=data_type, 
                                device=device)  
        
        t_tensor = t_tensor.view(2,1,1)  
        z_NODE_list.append(z_current.clone())
        z_whole = ode_trained(z_current, t_tensor, return_whole_sequence=True)
        z_next = z_whole[-1,0,:].unsqueeze(0)  
        z_current = z_next

        # Get true data
        z_real_inp = z_True_current
        z_real_next = rk4_step(z_real_inp,step_i*h_ode, h_ode)
        z_True_list.append(z_True_current.detach().clone())
        z_True_current = z_real_next
        step_i += 1
        
z_node_array = torch.cat(z_NODE_list, dim=0).cpu().numpy()  
z_true_array = torch.cat(z_True_list, dim=0).cpu().numpy()  # shape=(step_len,2)
"""
Time Response of Prediction 
"""
plt.figure(figsize=(10,4))
plt.subplot(1,2,1)
plt.plot(time_test, z_node_array[:,0],'m-',label="NODE $z_{1}$")
plt.plot(time_test, z_node_array[:,1],'c--',label="NODE $z_{2}$")
plt.plot(time_test[:-1], z_true_array[:,0],'r-',label="True $z_{1}$")
plt.plot(time_test[:-1], z_true_array[:,1],'b--',label="True $z_{2}$")
plt.xlabel('Time (sec)')
plt.grid(True)
plt.legend()
plt.show()

sio.savemat('Ex2_C2_NODE_trajectory.mat', {"data": z_node_array })
sio.savemat('Ex2_C2_True_trajectory.mat', {"data": z_true_array })
