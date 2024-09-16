
import numpy as np
import matplotlib.pyplot as plt
from casadi import *
import pandas as pd
import scipy as sci
import math
import numpy.linalg as LA
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
from matplotlib import patches
import os
import pandas as pd 

current_directory = os.getcwd()
final_directory = os.path.join(current_directory, r'output')
if not os.path.exists(final_directory):
   os.makedirs(final_directory)


class Obstacle:
    def __init__(self,x,y,r):
        self.x=x
        self.y=y
        self.r=r
       
class Pose:  
    def __init__(self,x,y,theta):
        self.x     = x
        self.y     = y
        self.theta = theta
        
class Prob: 
    def __init__(self):
        self.N        =  4
        self.nx       =  3
        self.nu       =  2
        self.dt       =  0.1
        self.xmin     = -5
        self.xmax     =  5
        self.ymin     = -5
        self.ymax     =  5
        self.thetamin = -np.pi
        self.thetamax =  np.pi
        self.obs      =  np.array([ Obstacle(  0,-3,  1), Obstacle( -3, 3,  1),  Obstacle(  3, 3,  1) ])
        self.ds       =  0.2


class Vehicle:
    
    def __init__(self, x, y, theta, v, w, r, prob, vminmax,wminmax,Q,R, optimisation_bool):
        self.x                  =  x
        self.y                  =  y
        self.theta              =  theta
        self.v                  =  v
        self.w                  =  w
        self.r                  =  r
        self.vmin               =  vminmax[0]
        self.vmax               =  vminmax[1]
        self.wmin               =  wminmax[0]
        self.wmax               =  wminmax[1]
        self.Q                  =  np.diag(Q)
        self.R                  =  np.diag(R)
        self.pred               =  np.array(  [np.array([x,y,theta])]*(prob.N+1)   ) 
        self.history            =  []
        self.optimisation_min   = optimisation_bool
           
    def position_update(self,pred_update):
        self.x      = pred_update[1,0]
        self.y      = pred_update[1,1]
        self.theta  = pred_update[1,2]      
        self.pred=pred_update



def solve(prob, vehicle, opponent_vehicle_pred):
    
    
    N=prob.N
    nx=prob.nx
    nu=prob.nu
    dt=prob.dt
    xmin=prob.xmin
    xmax=prob.xmax
    ymin=prob.ymin
    ymax=prob.ymax
    thetamin=prob.thetamin
    thetamax=prob.thetamax
    obs=prob.obs
    ds=prob.ds
    
    optimisation_min = vehicle[0].optimisation_min
    
    
    Nv = len(vehicle)
   
    print("solving")
    X=SX.sym('x',Nv*(N+1)*nx+Nv*N*nu  ,1 )
    
    "Kinematic constraints"
    constrain=vertcat()
    lb=vertcat()
    ub=vertcat()
    
    for j in range(Nv):
        ind  = j*nx*(N+1)
        ind1 = Nv*nx*(N+1) + j*nu*N
        for i in range(N):
          constrain=vertcat( constrain, X[ind+i*nx]   + dt * X[ind1+i*nu]   * SX.cos(X[ind+i*nx+2]) -  X[ind+(i+1)*nx]     )
          constrain=vertcat( constrain, X[ind+i*nx+1] + dt * X[ind1+i*nu]   * SX.sin(X[ind+i*nx+2]) -  X[ind+(i+1)*nx+1]   )
          constrain=vertcat( constrain, X[ind+i*nx+2] + dt * X[ind1+i*nu+1]                         -  X[ind+(i+1)*nx+2]   )
          lb=vertcat(lb,0)
          lb=vertcat(lb,0)
          lb=vertcat(lb,0)
          ub=vertcat(ub,0)
          ub=vertcat(ub,0)
          ub=vertcat(ub,0)
          
   
        
    "adding initial conditions"
    for j in range(Nv):
        ind  = j*nx*(N+1)
        constrain=vertcat(constrain, X[ind+0] )
        constrain=vertcat(constrain, X[ind+1] )
        constrain=vertcat(constrain, X[ind+2] )
        lb=vertcat(lb,vehicle[j].x)
        lb=vertcat(lb,vehicle[j].y)
        lb=vertcat(lb,vehicle[j].theta)
        ub=vertcat(ub,vehicle[j].x)
        ub=vertcat(ub,vehicle[j].y)
        ub=vertcat(ub,vehicle[j].theta)
    
   
    
    "adding obstacle constrain"
    for k in range(Nv):
        ind  = k*nx*(N+1)
        for j in range(np.size(obs)):
            for i in range( N+1):    
                constrain=vertcat(constrain, SX.sqrt(  ( X[ind+i*nx] - obs[j].x )**2 + ( X[ind+i*nx+1] - obs[j].y )**2    )   )
                lb=vertcat(lb, vehicle[k].r + obs[j].r + ds)
                ub=vertcat(ub,np.inf)
    
    
    
    # "adding self collision"
    # for k in range(Nv):
    #     ind1 =  k*nx*(N+1)
    #     for j in range(k+1,Nv):
    #         ind2 = j*nx*(N+1) 
    #         for i in range(N+1):
    #             constrain=vertcat(constrain, SX.sqrt(  ( X[ind1+i*nx] - X[ind2+i*nx] )**2 + ( X[ind1+i*nx+1] - X[ind2+i*nx+1] )**2    )   )
    #             lb=vertcat(lb, vehicle.r + vehicle.r  + ds)
    #             ub=vertcat(ub,np.inf)

    "Adding states min max constraint"
    for j in range(Nv):
        ind  = j*nx*(N+1)
        ind1 = Nv*nx*(N+1) + j*nu*N
        for i in range(N+1):
            constrain= vertcat(constrain, X[ind+i*nx])
            constrain= vertcat(constrain, X[ind+i*nx+1])
            constrain= vertcat(constrain, X[ind+i*nx+2])
            lb=vertcat(lb,xmin)
            lb=vertcat(lb,ymin)
            lb=vertcat(lb,thetamin)
            ub=vertcat(ub,xmax)
            ub=vertcat(ub,ymax)
            ub=vertcat(ub,thetamax)
            if i!=N:
                constrain=vertcat(constrain, X[ind1+i*nu])
                constrain=vertcat(constrain, X[ind1+i*nu+1])
                lb=vertcat(lb,vehicle[j].vmin)
                lb=vertcat(lb,vehicle[j].wmin)
                ub=vertcat(ub,vehicle[j].vmax)
                ub=vertcat(ub,vehicle[j].wmax)
    
    
    if optimisation_min:
        objective=SX(0)
        for j in range(Nv):
            ind  = j*nx*(N+1)
            ind1 = Nv*nx*(N+1) + j*nu*N
            for k in range(len(opponent_vehicle_pred)):
                for i in range(1,N+1):
                    objective = objective + vehicle[j].Q[0,0] * ( X[ind+i*nx]    - opponent_vehicle_pred[k][i,0]     )**2
                    objective = objective + vehicle[j].Q[1,1] * ( X[ind+i*nx+1]  - opponent_vehicle_pred[k][i,1]     )**2
                    objective = objective + vehicle[j].Q[2,2] * ( X[ind+i*nx+2]  - opponent_vehicle_pred[k][i,2]     )**2    
            for i in range(N):
                objective = objective + vehicle[j].R[0,0] * (X[ind1+i*nu])**2
                objective = objective + vehicle[j].R[1,1] * (X[ind1+i*nu+1])**2
    else:
        objective=SX(0)
        for j in range(Nv):
            ind  = j*nx*(N+1)
            ind1 = Nv*nx*(N+1) + j*nu*N
            for k in range(len(opponent_vehicle_pred)):
                for i in range(1,N+1):
                    objective = objective - vehicle[j].Q[0,0] * ( X[ind+i*nx]    - opponent_vehicle_pred[k][i,0]     )**2
                    objective = objective - vehicle[j].Q[1,1] * ( X[ind+i*nx+1]  - opponent_vehicle_pred[k][i,1]     )**2
                    objective = objective - vehicle[j].Q[2,2] * ( X[ind+i*nx+2]  - opponent_vehicle_pred[k][i,2]     )**2    
            for i in range(N):
                objective = objective - vehicle[j].R[0,0] * (X[ind1+i*nu])**2
                objective = objective - vehicle[j].R[1,1] * (X[ind1+i*nu+1])**2
    

    
    nlp      =  {}
    nlp['x'] =  X
    nlp['f'] =  objective
    nlp['g'] =  constrain
    opts = {}
    opts['ipopt.print_level'] = 0
    F        =  nlpsol('F','ipopt',nlp,opts)
    sol      =  F(ubg=ub,lbg=lb)
    
    out = []
    for j in range(Nv):
        ind1 =  Nv * nx * (N+1)  +     j * nu * N
        ind2 =  Nv * nx * (N+1)  + (j+1) * nu * N
        out.append( sol['x'][ind1:ind2] )
    
   
    return out


def state_calc(vehicle, inp,prob):
    pred      = []
    for j in range(len(vehicle)):
        x=vehicle[j].x
        y=vehicle[j].y
        theta=vehicle[j].theta
        pred.append( np.array([[ x, y, theta]]) )
        for i in range(prob.N):
             x         =   x     + float(inp[j][i*prob.nu])   * np.cos(theta) * prob.dt
             y         =   y     + float(inp[j][i*prob.nu])   * np.sin(theta) * prob.dt
             theta     =   theta + float(inp[j][i*prob.nu+1]) * prob.dt
             pred[j]  = np.vstack((  pred[j] , np.array([[ x, y, theta ]])      ))
    return pred


def pred_form(vehicle):
    pred=[]
    for i in range(len(vehicle)):
        pred.append(vehicle[i].pred)
    return pred

if __name__ == "__main__":
    
    prob=Prob()
    threshold=0.2
    ite=0
    max_ite=200
   
    
    n_trial=5
    x_trial = np.linspace(-5,5,n_trial)
    y_trial = np.linspace(-5,5,n_trial)
    pts_trial = np.empty((0,2),dtype=object)
    result=np.empty((0,3),dtype=object)
   
    for i in range(n_trial):
        for j in range(n_trial):
            pts_trial = np.vstack((pts_trial,np.array([[x_trial[i], y_trial[j]]]) ))
    
    for trial in range(len(pts_trial)):
        terminated = False
        threshold_stop_condition= False
        
        vehicle_pursuer  = [    Vehicle( -4.5, -4,  np.pi/2, 0, 0, 0.2 , prob, [-2,2], [-2,2], [10,8,4], [1,1], True   ),
                            Vehicle(  4.5, -4,  np.pi/2, 0, 0, 0.2 , prob, [-2,2], [-2,2], [10,8,4], [1,1], True   ),
                            Vehicle( -4.5,  4, -np.pi/2, 0, 0, 0.2 , prob, [-2,2], [-2,2], [10,8,4], [1,1], True   ),
                            Vehicle(  4.5,  4, -np.pi/2, 0, 0, 0.2 , prob, [-2,2], [-2,2], [10,8,4], [1,1], True   )      ]
    
        
        vehicle_evader   = [    Vehicle(    pts_trial[trial,0],  pts_trial[trial,1], -np.pi/2, 0, 0, 0.2 , prob, [-3,3], [-2,2], [ 8,6,4], [1,1], False  )       ]
       
        
        "checking current position of the vehicle"
        check_pos_cond= False
        for i in range(len(prob.obs)):
            dist = LA.norm([  pts_trial[trial,0] - prob.obs[i].x ,  pts_trial[trial,1] - prob.obs[i].y   ])
            if dist <= prob.obs[i].r + prob.ds:
                check_pos_cond = True
                break;
        
        if check_pos_cond:
            continue
        
        check_pos_cond= False
        for i in range(len(vehicle_pursuer)):
            dist = LA.norm( [ vehicle_pursuer[i].x  - pts_trial[trial,0] , vehicle_pursuer[i].y  - pts_trial[trial,1]         ])
            if dist <=vehicle_pursuer[i].r+ vehicle_evader[0].r:
                check_pos_cond = True
                break;
        
        if check_pos_cond:
            continue
        
        
        "Variables saves value for plotting "    
        evader=[]
        pursuer =[]
        for i in range(len(vehicle_evader)):
            evader.append(      np.array([[   vehicle_evader[i].x,    vehicle_evader[i].y ,  vehicle_evader[i].theta     ]])     )
        for i in range(len(vehicle_pursuer)) : 
            pursuer.append(     np.array([[   vehicle_pursuer[i].x,   vehicle_pursuer[i].y,  vehicle_pursuer[i].theta    ]])     )
            
        while True:
            
            inp              = solve(prob,vehicle_evader,pred_form(vehicle_pursuer))
            pred             = state_calc(vehicle_evader, inp, prob)
            inp              = solve(prob, vehicle_pursuer, pred )
            pred_pursuer     = state_calc(vehicle_pursuer,inp, prob)
        
            inp              = solve(prob, vehicle_pursuer, pred_form(vehicle_evader) )
            pred             =  state_calc(vehicle_pursuer,inp, prob)
            inp              = solve(prob, vehicle_evader, pred)
            pred_evader      = state_calc(vehicle_evader, inp, prob)
            
            "position update"
            for i in range(len(vehicle_evader)):
                vehicle_evader[i].position_update(pred_evader[i])
                evader[i]  = np.vstack((   evader[i],     np.array( [[    vehicle_evader[i].x,   vehicle_evader[i].y,   vehicle_evader[i].theta    ]]  )   ))
            for i in range(len(vehicle_pursuer)):
                vehicle_pursuer[i].position_update(pred_pursuer[i])
                pursuer[i] = np.vstack((   pursuer[i],    np.array( [[    vehicle_pursuer[i].x,  vehicle_pursuer[i].y,  vehicle_pursuer[i].theta   ]]  )   ))
        
            for i in range(len(vehicle_evader)):
                    for j in range(len(vehicle_pursuer)):
                        temp_dist= LA.norm([ vehicle_evader[i].x-vehicle_pursuer[j].x, vehicle_evader[i].y-vehicle_pursuer[j].y  ])
                        print("=================================> dist ", temp_dist)
                        if temp_dist <=threshold:
                            print("TERMINATED")
                            threshold_stop_condition=True
                            terminated = True
                            break;
                
            if threshold_stop_condition:
                break
            
            ite=ite+1
            if ite>=max_ite:
                break
        
        
        
        "ANIMATION WRITING AND VISUALISATION"

        fig_anim, ax_anim = plt.subplots(figsize=(6,6), dpi=200)
        
        obs_circle=[]
        for j in range(len(prob.obs)):
            obs_circle.append( np.column_stack((   prob.obs[j].r*np.cos(np.linspace(0,2*np.pi,100))+prob.obs[j].x , prob.obs[j].r*np.sin(np.linspace(0,2*np.pi,100))+prob.obs[j].y                ))     )
        
        evader_circle=[]
        pursuer_circle=[]
        
        for i in range(len(vehicle_evader)):
            evader_circle.append( np.column_stack((  vehicle_evader[i].r*np.cos(np.linspace(0,2*np.pi,100))  ,  vehicle_evader[i].r*np.sin(np.linspace(0,2*np.pi,100))      )) )
        
        for i in range(len(vehicle_pursuer)):
            pursuer_circle.append( np.column_stack((  vehicle_pursuer[i].r*np.cos(np.linspace(0,2*np.pi,100))  ,  vehicle_pursuer[i].r*np.sin(np.linspace(0,2*np.pi,100))      )) )
        
        
        def funcAnim(i):
            plt.cla()
            
            plt.xlim(-6,6)
            plt.ylim(-6,6)
            ax_anim.set_aspect("equal")
          
            for j in range(len(obs_circle)):
                ax_anim.plot(obs_circle[j][:,0], obs_circle[j][:,1], color="blue")
            
            for j in range(len(evader)): 
                ax_anim.plot( evader[j][:i+1,0],  evader[j][:i+1,1],  color="lightsalmon", label="Evader"  )
                ax_anim.plot( evader_circle[j][:,0]  + evader[j][i,0] , evader_circle [j][:,1]  + evader[j][i,1]  ,   color="lightsalmon"  )
                plt.arrow(  evader[j][i,0] ,  evader[j][i,1], vehicle_evader[j].r*np.cos( evader[j][i,2] ),   vehicle_evader[j].r*np.sin(  evader[j][i,2] )  , head_width = 0.2,  width = 0.01, ec ='black', fc='black'  )
                
            for j in range(len(pursuer)):
                ax_anim.plot( pursuer[j][:i+1,0],  pursuer[j][:i+1,1], color="palegreen", label="Pursuer" )
                ax_anim.plot( pursuer_circle[j][:,0]  + pursuer[j][i,0] , pursuer_circle[j][:,1]  + pursuer[j][i,1] ,     color="palegreen"  )
                plt.arrow(  pursuer[j][i,0],  pursuer[j][i,1], vehicle_pursuer[j].r*np.cos( pursuer[j][i,2] ), vehicle_pursuer[j].r*np.sin(  pursuer[j][i,2] ), head_width = 0.2,  width = 0.01, ec ='black', fc='black'  )
            
            
            ax_anim.add_patch(patches.Rectangle((-5, -5), 10, 10, edgecolor='orange',   facecolor='none', linewidth=2))
            
            
            return
        
        anim = FuncAnimation( fig_anim, funcAnim, frames = np.arange(len(evader[0])) ,interval=10, repeat=True)    
        
        
        writer = PillowWriter(fps=6)
        name ="x_"+str(pts_trial[trial,0])+"_y_"+str(pts_trial[trial,1])
        anim.save("output\ " + name + ".gif", writer=writer)
        
        result = np.vstack((result, np.array([[pts_trial[trial,0],pts_trial[trial,1],terminated]])))
    

pd.DataFrame(result).to_csv("output/result.csv")








































