# Point-mass Runner
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP

import sys
import math
import numpy as np
from casadi import *
from random import random


N = 100 # number of control intervals
opti = Opti() # optimization problem

# ---- problem parameters ---------
M     = 80.00  # [kg  ] body mass
g     =  9.81  # [m/s2] gravitational acc
L     =  1.00  # [m   ] leg length
v_dim =  3.00  # [m/s ] speed
f_dim =  3.00  # [Hz  ] step frequency

d_dim = v_dim/f_dim  # [m ] step length
# Vo_dim= -sqrt(2*g*H_dim) # [m/s ] dimensional velocity at initial contact
# Tf_dim= -2*Vo_dim/g      # [s   ] flight time 
tnd   =  sqrt(L/g)       # [s   ] time non-dimensional factor

f = opti.parameter(); opti.set_value(f ,  f_dim*tnd  ) # step frequency
d = opti.parameter(); opti.set_value(d ,  d_dim/L    ) # step length
T = 1/f  # [nd] total time duration


# ---- decision variables ---------
q = opti.variable(6,N+1) # state trajectory
u = opti.variable(5,N  ) # control var
Tc= opti.variable()      # final time
xf= opti.variable()      # foot contact position
xb  = q[0,:] # hor body pos
xbd = q[1,:] # hor body vel
yb  = q[2,:] # vrt body pos
ybd = q[3,:] # vrt body vel
F   = q[4,:] # leg force
Fd  = q[5,:] # leg force rate
Fdd = u[0,:] # leg force rate2
pP  = u[1,:] # pos leg power
qP  = u[2,:] # neg leg power
pFdd= u[3,:] # pos force rate2
qFdd= u[4,:] # neg force rate2

Ll = sqrt((xb-xf)**2+yb**2)   # leg length
Lld= ((xb-xf)*xbd+yb*ybd)/Ll  # leg velocity
cTh= (xb-xf)/Ll # cos(Th) where Th is + ccw from -->
sTh= (yb   )/Ll # sin(Th) where Th is + ccw from -->



dt = Tc/N # length of a control interval


# ---- dynamic constraints --------
qd = lambda q,u,xf: vertcat(  q[1]                                      , # xbd 
                             (q[0]-xf)/sqrt((q[0]-xf)**2+q[2]**2)*q[4]  , # xbdd
                              q[3]                                      , # ybd
                             (q[2]   )/sqrt((q[0]-xf)**2+q[2]**2)*q[4]-1, # ybdd
                              q[5]                                      , # Fd
                              u[0]                                      ) # Fdd

for k in range(N): # loop over control intervals
   # Runge-Kutta 4 integration
   k1 = qd(q[:,k]         , u[:,k] , xf )
   k2 = qd(q[:,k]+dt/2*k1 , u[:,k] , xf )
   k3 = qd(q[:,k]+dt/2*k2 , u[:,k] , xf )
   k4 = qd(q[:,k]+dt*k3   , u[:,k] , xf )
   x_next = q[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
   opti.subject_to(q[:,k+1]==x_next) # close the gaps


P= F*Lld # leg power

# ---- Integrate for Objective Function --------
Int_pP   = lambda              pP:  pP         # (+) power
Int_qP   = lambda              qP:  qP         # (-) power
Int_Fdd  = lambda       pFdd,qFdd:  pFdd+qFdd  # Fdd
Int_comp = lambda pP,qP,pFdd,qFdd:  pP*qP + pFdd*qFdd # comlimentary constraints
Cum_pP=0; Cum_qP=0; Cum_Fdd=0; Cum_comp=0
for k in range(1,N): # loop over control intervals
   Cum_pP   = Cum_pP   + dt*(  Int_pP(pP[:,k-1]) + Int_pP(pP[:,k]) )/2
   Cum_qP   = Cum_qP   + dt*(  Int_pP(qP[:,k-1]) + Int_pP(qP[:,k]) )/2
   Cum_Fdd  = Cum_Fdd  + dt*( Int_Fdd(pFdd[:,k-1],qFdd[:,k-1]) + Int_Fdd(pFdd[:,k  ],qFdd[:,k  ]) )/2
   Cum_comp = Cum_comp + dt*( Int_comp(pP[:,k-1],qP[:,k-1],pFdd[:,k-1],qFdd[:,k-1]) + \
                              Int_comp(pP[:,k  ],qP[:,k  ],pFdd[:,k  ],qFdd[:,k  ]) )/2

# ---- objective ------------------
opti.minimize(Cum_pP/0.25 + Cum_qP/1.20 + 5e-3*Cum_Fdd + Cum_comp) # minimize objective function


# ---- path constraints -----------
opti.subject_to(  yb>=0)  # body above ground
opti.subject_to(   F>=0)  # extension force only
opti.subject_to(  pP>=0)  # bounded control
opti.subject_to(  qP>=0)  # bounded control
opti.subject_to(pFdd>=0)  # bounded control
opti.subject_to(qFdd>=0)  # bounded control
opti.subject_to(P[0:N] ==pP  -qP  )  # power slack vars
opti.subject_to(  Fdd  ==pFdd-qFdd)  # Fdd   slack vars
opti.subject_to(  opti.bounded(0,Ll,1)  )  # max leg length constraint


# ---- boundary conditions --------
opti.subject_to(xb[ 0]== 0) # start at pos 0
opti.subject_to( F[ 0]== 0) # start w zero leg force
opti.subject_to( F[-1]== 0) # end   w zero leg force
opti.subject_to(Tc == T-(ybd[-1] - ybd[0])) # enforce task frequency
opti.subject_to( d == xb[-1] + xbd[-1]*(ybd[-1] - ybd[0])) # body travels step length

# ---- continuity conditions --------
opti.subject_to(xbd[0]==xbd[-1])
opti.subject_to( yb[0]== yb[-1] + ybd[-1]*(ybd[-1]-ybd[0]) - 0.5*(ybd[-1]-ybd[0])**2)
# opti.subject_to(ybd[0]==ybd[-1])


# ---- misc. constraints  ----------
opti.subject_to(Tc<=T) # time of contact <= than step time

# ---- initial values for solver ---
opti.set_initial(xb , random())
opti.set_initial(xbd, random())
opti.set_initial(yb , random())
opti.set_initial(ybd, random())
#opti.set_initial(Tc, 1)


# ---- solve NLP -------------------
opti.solver("ipopt") # set numerical backend
sol = opti.solve()   # actual solve


# ---- outputs ----------------------------
print("Look!")
print(sol.value(xf)*L)
print([sol.value( xb[0])*L     , sol.value( xb[-1])*L    ])
print([sol.value(xbd[0])*L/tnd , sol.value(xbd[-1])*L/tnd])
print([sol.value( yb[0])*L     , sol.value( yb[-1])*L    ])
print([sol.value(ybd[0])*L/tnd , sol.value(ybd[-1])*L/tnd])
print([sol.value(T)*tnd, sol.value(Tc)*tnd, sol.value((ybd[-1] - ybd[0]))*L/tnd, sol.value(d)*L])
print([sol.value(Cum_pP), sol.value(Cum_qP), sol.value(Cum_Fdd), sol.value(Cum_comp)])


# ---- construct time vector --------------
tq= np.linspace(0,sol.value(Tc),N+1) # time vector for states
tu= np.linspace(0,sol.value(Tc)-sol.value(Tc)/N,N) # time vector for controls


# ---- post-processing ------------
from pylab import plot, step, figure, legend, show, spy

plot(tq,sol.value(xb ),label="xb(t)")
plot(tq,sol.value(xbd),label="xbd(t)")
legend(loc="upper left")
plot(tq,sol.value(yb ),label="yb(t)")
plot(tq,sol.value(ybd),label="ybd(t)")
legend(loc="upper left")
figure()
plot(tq,sol.value(F ),label="F(t)")
legend(loc="upper left")
figure()
plot(tq,sol.value(Fd),label="Fd(t)")
legend(loc="upper left")
figure()
plot(tu,sol.value(Fdd),label="Fdd(t)")
legend(loc="upper left")

figure()
plot(tq, sol.value( P),label="P(t)")
plot(tu, sol.value(pP),label="P(+)")
plot(tu,-sol.value(qP),label="P(-)")
legend(loc="upper left")

#figure()
#spy(sol.value(jacobian(opti.g,opti.x)))
#figure()
#spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))

show()
