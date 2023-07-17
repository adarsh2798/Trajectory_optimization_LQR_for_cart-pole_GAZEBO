import numpy as np

from scipy import linalg
from control import *

m1 = 2.7 
m2 = 0.86 
J = 0.076 
l = 0.5 
g=9.8


alpha = (m1 + m2)*(J +m2*l**2)- m2**2*l**2
A = np.array([ 
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [0, -g*m2**2*l**2/alpha, 0, 0],
                [0, m2*g*l*(m1 + m2)/alpha, 0, 0]
            ])

B = np.array([
                [0],
                [0],
                [(J + m2*l**2)/alpha],
                [-m2*l/alpha]
            ])
#Q = np.diag([903000, 913000, 18800000000, 60470000000]) # <----good1 
#Q = np.diag([1,100,0.0000001,1000])# <-----good2 

#Q = np.diag([0.1,100,0.0000001/1000,1000000*1000]) # good3
#R = 0.01*np.eye(1)                                 #good3

Q = np.diag([0.1,1000,0.0000001/1000,1000000*1000]) 
R = 0.01*np.eye(1)                                 


#Q = np.diag([1000000000, 2000, 1000000000, 10000000000]) 
def compute_K():
   global A,B,Q,R
   K,S,P=lqr(A,B,Q,R)
   print(K[0])
   return K[0]
