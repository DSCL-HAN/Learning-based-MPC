import math
import numpy as np 
import matplotlib
import matplotlib.pyplot as plt
import torch
from torch import nn
from torch.nn import functional as F
import torch.version
from NODE_helper import*
import scipy.io

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

#################################################
# 1) Spiral Trajectory & Runge-Kutta Order 4
#################################################
def Spiral_Trajectory(z, t):
    dz = torch.zeros_like(z)
    dz[:,0] = -0.1*z[:,0] - 1.0*z[:,1]
    dz[:,1] =  1.0*z[:,0] - 0.1*z[:,1]
    
    return dz

def rk4_step(z, t, h):
    k1 = Spiral_Trajectory(z, t)*h
    k2 = Spiral_Trajectory(z + 0.5*k1, t + 0.5*h)*h
    k3 = Spiral_Trajectory(z + 0.5*k2, t + 0.5*h)*h
    k4 = Spiral_Trajectory(z + k3,     t + h)*h

    return z + ((k1+k4)/6+(k2+k3)/3)

#################################################
# 2) Recurrent Neural Network
#################################################
class SimpleRNN(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super().__init__()
        self.rnn = nn.RNN(input_dim, hidden_dim, batch_first=True)
        self.lin = nn.Linear(hidden_dim, output_dim)

    def forward(self, z_seq):
        # z_seq: (batch, seq_len, input_dim)
        out, h_n = self.rnn(z_seq)  
        y = self.lin(out)  
        return y, h_n  

#################################################
# 3) Main Code for Training Neural ODE & RNN
#################################################
def main():
    """
    Training for Neural ODE
    """
    Tf = 20
    n_points = 200
    h_ode = 0.05
    z_initial = [0.1, 0.0]
    n_steps = 500
    min_delta_time = 1
    max_delta_time = 10
    Nd = 50
    z0 = Variable(torch.Tensor([z_initial])).to(device=device, dtype=data_type) 

    index_np = np.arange(0, n_points, 1, dtype=int)
    index_np = np.hstack([index_np[:, None]])
    times_np = np.linspace(0, Tf, num=n_points)
    times_np = np.hstack([times_np[:, None]])

    times = torch.from_numpy(times_np[:, :, None]).to(z0).to(device=device, dtype=data_type) 
    ode_true = NeuralODE(SpiralFunctionExample())
    obs = ode_true(z0, times, return_whole_sequence=True).detach() # obs size is n_points

    ode_true = NeuralODE(SpiralFunctionExample())
    ode_trained = NeuralODE(RandomLinearODEF())
    plot_freq = 10

    print("===== Train Neural ODE =====")

    obs_temp_out, ts_temp_out, Loss_NODE_out = conduct_experiment(
                                                ode_true, ode_trained,
                                                n_steps = n_steps,
                                                name = "linear",
                                                plot_freq=plot_freq,
                                                Tf = Tf,           
                                                n_points = n_points,
                                                z_initial = z_initial,
                                                min_delta_time = min_delta_time,
                                                max_delta_time = max_delta_time,
                                                Nd = Nd)
    PATH = "./node_model.pt"

    ## Trained Neural ODE Data Save
    # torch.save(ode_trained.state_dict(), PATH)
    # scipy.io.savemat('Loss_NODE.mat', {"data": Loss_NODE_out })

    ## Trained Neural ODE Data Load
    Loss_NODE = scipy.io.loadmat('Loss_NODE.mat')
    Loss_NODE = Loss_NODE['data']
    ckpt = torch.load(PATH)
    ode_trained.load_state_dict(ckpt)
    ode_trained.eval()

    """
    Training for RNN
    """
    # Get trajectory of random timespan 
    def create_batch_h():
        t0 = np.random.uniform(0, Tf - max_delta_time)
        t1 = t0 + np.random.uniform(min_delta_time, max_delta_time)
        idx = sorted(np.random.permutation(index_np[(times_np > t0) & (times_np < t1)])[:Nd])
        # 0 ~ Nd, the max number of idx is max_points_num (Nd)
        
        obs_ = obs[idx] # original number of "obs" is n_points
        ts_ = times[idx]
        # size of "obs_" and "ts_" is same

        return obs_, ts_
    
    input_dim_RNN = 2
    hidden_dim_RNN = 16
    output_dim_RNN = 2

    rnn_model = SimpleRNN(input_dim_RNN, hidden_dim_RNN, output_dim_RNN).to(device)
    optimizer = torch.optim.Adam(rnn_model.parameters(), lr=1e-2)

    print("===== Train RNN =====")
    loss_list_RNN = []  
    for step in range(n_steps):
        obs_, ts_ = create_batch_h()
        obs_ = obs_.to(device=device, dtype=data_type)
        obs_ = obs_.squeeze(1)  
        obs_ = obs_.unsqueeze(0) 

        pred_seq, h_ = rnn_model(obs_)              
        pred_next = pred_seq[:, :-1, :]
        true_next = obs_[:, 1:, :]

        loss = F.mse_loss(pred_next, true_next)
        loss_list_RNN.append(loss.item())  

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if step % 200 == 0:
            print(f"[RNN][Step {step}] Loss={loss.item():.6f}")

#################################################
# 4) Prediction using Neural ODE & RNN
#################################################
    """
    Prediction Neural ODE
    """
    time_test = np.arange(0, Tf+h_ode, h_ode)
    z_init = torch.tensor([z_initial], dtype=data_type, device=device) 
    z_NODE_list = []
    z_current = z_init.clone()

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
    
    z_node_array = torch.cat(z_NODE_list, dim=0).cpu().numpy()  

    """
    Prediction RNN
    """
    h_ode = torch.tensor(h_ode, dtype=data_type, device=device) 
    hidden = torch.zeros(1, 1, hidden_dim_RNN, device=device)
    step_i = torch.tensor(0, device=device)
    
    z_RNN_list = []
    z_RNN_current = z_init.clone()
    z_True_list = []
    z_True_current = torch.tensor([z_initial], dtype=data_type, device=device) 

    with torch.no_grad():
        for i in range(len(time_test)):
            # Get predicted data using RNN
            z_inp = z_RNN_current.unsqueeze(1)
            out, hidden = rnn_model.rnn(z_inp, hidden) 
            z_next = rnn_model.lin(out)    
            z_RNN_list.append(z_RNN_current.detach().clone())
            z_RNN_current = z_next[:, -1, :]
            
            # Get true data
            z_real_inp = z_True_current
            z_real_next = rk4_step(z_real_inp,step_i*h_ode, h_ode)
            z_True_list.append(z_True_current.detach().clone())
            z_True_current = z_real_next
            step_i += 1
            
    z_rnn_array = torch.cat(z_RNN_list, dim=0).cpu().numpy()  # shape=(step_len,2)
    z_true_array = torch.cat(z_True_list, dim=0).cpu().numpy()  # shape=(step_len,2)

#################################################
# 5) Plot 
#################################################   
    """
    Loss Function
    """
    plt.figure(figsize=(10,4))
    plt.plot(loss_list_RNN,  'b-', label="Loss RNN")
    plt.plot(Loss_NODE[0,:], 'r-', label="Loss NODE")
    plt.xlabel("Train Step")
    plt.ylabel("MSE Loss")
    plt.grid(True)
    plt.legend()
    plt.show()
    """
    Time Response of Prediction 
    """
    plt.figure(figsize=(10,4))
    plt.subplot(1,2,1)
    plt.plot(time_test, z_rnn_array[:,0], 'k-', label="RNN $z_{1}$")
    plt.plot(time_test, z_rnn_array[:,1], 'g--', label="RNN $z_{2}$")
    plt.plot(time_test, z_node_array[:,0],'m-',label="NODE $z_{1}$")
    plt.plot(time_test, z_node_array[:,1],'c--',label="NODE $z_{2}$")
    plt.plot(time_test, z_true_array[:,0],'r-',label="True $z_{1}$")
    plt.plot(time_test, z_true_array[:,1],'b--',label="True $z_{2}$")
    plt.xlabel('Time (sec)')
    plt.grid(True)
    plt.legend()
    """
    Trajectories
    """
    plt.tight_layout()
    plt.subplot(1,2,2)
    plt.scatter(z_rnn_array[:,0], z_rnn_array[:,1],
                c='g', marker='o', label="RNN")
    plt.scatter(z_node_array[:,0], z_node_array[:,1],
                c='m', marker='x', label="NODE")
    plt.scatter(z_true_array[:,0], z_true_array[:,1],
                c='b', marker='^', label="True")
    plt.xlabel("$z_{1}(t)$")
    plt.ylabel("$z_{2}(t)$")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
