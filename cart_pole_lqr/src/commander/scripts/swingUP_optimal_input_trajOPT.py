import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
g=9.8
T=4 #final time 
N=20 # no. of collocation points
t_d=np.linspace(0,T,N) # discrete time steps
d_bar=0.3# final cart x-pos after top reached
tau_max=5 # max input to base joint
d_max=0.5 # max cart x- deviation allowed



l=0.5
m1=2.7
m2=0.86

def objective(tau):
  sum=0
  for i in range(N-1):
    h_k=t_d[i+1]-t_d[i]
    sum+=(h_k/2)*(tau[i]**2 +tau[i+1]**2)
    return sum


def x_dot_dynamics(x_k,theta_k,x_dot_k,theta_dot_k,tau_k):
  
  denom=m1+m2*(np.sin(theta_k))**2
  num=(     (l*m2*np.sin(theta_k)*theta_dot_k**2) 
             
              +(tau_k) 
             
              + (m2*g*np.cos(theta_k)*np.sin(theta_k)) )
  return num/denom
  
def theta_dot_dynamics(x_k,theta_k,x_dot_k,theta_dot_k,tau_k):
  
  denom=(m1+m2*(np.sin(theta_k))**2)*l
  num=(    (l*m2*np.cos(theta_k)*np.sin(theta_k)*theta_dot_k**2)
           
            +( tau_k*np.cos(theta_k))
            
            +(g*np.sin(theta_k)*(m1+m2))   )
  return -num/denom
           


def constraints(var):
 x,theta,x_dot,theta_dot,tau=np.split(var,5)

 x_dot_dynamics_array=x_dot_dynamics(x,theta,x_dot,theta_dot,tau)
 theta_dot_dynamics_array=theta_dot_dynamics(x,theta,x_dot,theta_dot,tau)
 h_k=t_d[3]-t_d[2]
 cons=np.zeros((4*(N-1)+8,))
 cons[:N-1]=x[1:]-x[:-1]-(h_k/2)*(x_dot[1:]+x_dot[:-1])
 cons[N-1:2*(N-1)]=theta[1:]-theta[:-1]-(h_k/2)*(theta_dot[1:]+theta_dot[:-1])
 cons[2*(N-1):3*(N-1)]=x_dot[1:]-x_dot[:-1]-(h_k/2)*(x_dot_dynamics_array[1:]+x_dot_dynamics_array[:-1])
 cons[3*(N-1):4*(N-1)]=theta_dot[1:]-theta_dot[:-1]-(h_k/2)*(theta_dot_dynamics_array[1:]+theta_dot_dynamics_array[:-1])
 cons[4*(N-1)] = x[0]
 cons[4*(N-1) + 1] = theta[0] 
 cons[4*(N-1) + 2] = x_dot[0] 
 cons[4*(N-1) + 3] = theta_dot[0]
 cons[4*(N-1) + 4]=x[N-1]-d_bar
 cons[4*(N-1) + 5]=theta[N-1]-np.pi 
 cons[4*(N-1) + 6]=x_dot[N-1]
 cons[4*(N-1) + 7]=theta_dot[N-1]
 return cons


def compute_input_swingUP():
   x_guess=t_d*(d_bar)/T
   theta_guess=(t_d/T)*np.pi
   x_dot_guess=np.zeros_like(x_guess)
   theta_dot_guess=np.zeros_like(theta_guess)
   tau_guess=np.zeros((N,))

   initial_guess=np.concatenate((x_guess,theta_guess,x_dot_guess,theta_dot_guess,tau_guess))

   bounds = [(-d_max, d_max)] * N + [(None, None)] * N + [(None, None)] * N + [(None, None)] * N + [(-tau_max, tau_max)] * N



   constraint_eq = {'type': 'eq', 'fun': constraints}
   result = minimize(objective, initial_guess, method='SLSQP', constraints=constraint_eq, bounds=bounds)
   return result.x[4*N:]





def get_interpolated_u_swingUP(t_curr,u_d):
     if(t_curr>T):
          return 0
     if(t_curr==0):
          return 0
     else:
         s=0
         e=N-1
         interval=[None,None]
         while(s<e):
           mid=int((s+e)/2)
           if(t_curr<=t_d[mid]):
               e=mid
           elif (t_curr>t_d[mid]):
                s=mid+1
 
         interval=[t_d[s-1],t_d[e]]
         tau=t_curr-interval[0]
         h_k=t_d[e]-t_d[s-1]
         input_interpolated=u_d[s-1] + (tau)*(u_d[e]-u_d[s-1])/h_k
         return input_interpolated
         
    
       
      



"""

############################# HERE THE DISCRETE INPUT AND STATE TRAJECTORIES ARE INTERPOLATED TO GET CONTINUOUS TRAJECTORIES######################################################
t_c=np.linspace(0,2,200)
input_opt_c=np.zeros_like(t_c)
phi_opt_c=np.zeros_like(t_c)
theta_opt_c=np.zeros_like(t_c)
phi_dot_opt_c=np.zeros_like(t_c)
theta_dot_opt_c=np.zeros_like(t_c)


for i in range(t_d.shape[0]-1):
  mask=(t_c>=t_d[i]) &(t_c<=t_d[i+1])
  t_interval=t_c[mask]
  tau=t_interval-t_d[i]
  h_k=t_d[3]-t_d[2]


  input_interval=tau_opt[i] + (tau)*(tau_opt[i+1]-tau_opt[i])/h_k
  phi_interval=phi_opt[i] +  phi_dot_opt[i]*tau + (tau**2/(2*h_k))*(phi_dot_opt[i+1]-phi_dot_opt[i])
  theta_interval=theta_opt[i] +  theta_dot_opt[i]*tau + (tau**2/(2*h_k))*(theta_dot_opt[i+1]-theta_dot_opt[i])
  phi_dot_interval=phi_dot_opt[i] +  phi_dot_dynamics(phi_opt[i],theta_opt[i],phi_dot_opt[i],theta_dot_opt[i],tau_opt[i])*tau + (tau**2/(2*h_k))*(phi_dot_dynamics(phi_opt[i+1],theta_opt[i+1],phi_dot_opt[i+1],theta_dot_opt[i+1],tau_opt[i+1])-phi_dot_dynamics(phi_opt[i],theta_opt[i],phi_dot_opt[i],theta_dot_opt[i],tau_opt[i]))
  theta_dot_interval=theta_dot_opt[i] +  theta_dot_dynamics(phi_opt[i],theta_opt[i],phi_dot_opt[i],theta_dot_opt[i],tau_opt[i])*tau + (tau**2/(2*h_k))*(theta_dot_dynamics(phi_opt[i+1],theta_opt[i+1],phi_dot_opt[i+1],theta_dot_opt[i+1],tau_opt[i+1])-theta_dot_dynamics(phi_opt[i],theta_opt[i],phi_dot_opt[i],theta_dot_opt[i],tau_opt[i]))

  input_opt_c[mask]=input_interval
  phi_opt_c[mask]=phi_interval
  theta_opt_c[mask]=theta_interval
  phi_dot_opt_c[mask]=phi_dot_interval
  theta_dot_opt_c[mask]=theta_dot_interval
##########################################################################################################################################################################################
"""
"""
plt.figure(figsize=(10, 6))
plt.plot(t_c, phi_opt_c, 'r', label='phi')
plt.plot(t_c, theta_opt_c, 'g', label='theta')
plt.plot(t_c, input_opt_c, 'b', label='input')
plt.scatter(t_d, phi_opt,s=20,c='red',marker='o')
plt.scatter(t_d, theta_opt,s=20,c='green',marker='o')
plt.scatter(t_d, tau_opt,s=20,c='blue',marker='o')
plt.xlabel('t')
plt.ylabel('Value')
plt.title('Optimized Variables')
plt.legend()
plt.grid(True)
plt.show()
"""

