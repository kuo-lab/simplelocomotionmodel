# Point-mass Hopper
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP

import sys
import numpy as np
from casadi import *


N = 100 # number of control intervals
opti = Opti() # optimization problem

# ---- problem parameters ---------
M     = 80.00            # [kg  ] body mass
g     =  9.81            # [m/s2] gravitational acc
L     =  1.00            # [m   ] leg length
f_dim =  2.00            # [Hz  ] frequency
H_dim =  0.15            # [m   ] height

Vo_dim= -sqrt(2*g*H_dim) # [m/s ] dimensional velocity at initial contact
Tf_dim= -2*Vo_dim/g      # [s   ] flight time 
tnd   =  sqrt(L/g)       # [s   ] time non-dimensional factor

f = opti.parameter(); opti.set_value(f ,  2.00*tnd  ) # frequency
Vo= opti.parameter(); opti.set_value(Vo, -1.00*tnd/L) # speed at initial contact
T = 1/f        # [nd] total time duration
Tf= Tf_dim/tnd # [nd] flight time


# ---- decision variables ---------
q = opti.variable(3,N+1) # state trajectory
y = q[0,:] # vrt pos
yd= q[1,:] # vrt vel
F = q[2,:] # leg force
u = opti.variable(1,N) # control var
Fd= u[0,:]             # leg force rate
Tc = opti.variable()   # final time


dt = Tc/N # length of a control interval


# ---- dynamic constraints --------
qd = lambda q,u: vertcat(q[1],q[2]-1,u[0]) # dq/dt = f(q,u)

for k in range(N): # loop over control intervals
   # Runge-Kutta 4 integration
   k1 = qd(q[:,k]         , u[:,k])
   k2 = qd(q[:,k]+dt/2*k1 , u[:,k])
   k3 = qd(q[:,k]+dt/2*k2 , u[:,k])
   k4 = qd(q[:,k]+dt*k3   , u[:,k])
   x_next = q[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
   opti.subject_to(q[:,k+1]==x_next) # close the gaps


P= F*yd # leg power
Int = lambda u: vertcat(u**2) # J = u^2
cumInt= 0
# ---- Integrate for Objective Function --------
for k in range(N): # loop over control intervals
   # Rectangle Rule 4 integration
   cumInt = cumInt + dt*(Int(u[:,k]))


# ---- objective ------------------
opti.minimize(cumInt) # minimize objective function


# ---- path constraints -----------
opti.subject_to(y>=0)   # body above ground
opti.subject_to( opti.bounded(0,F,5) )  # bounded control

# ---- boundary conditions --------
opti.subject_to( y[ 0]== L   ) # start w straight leg
opti.subject_to( y[-1]== L   ) # end   w straight leg
opti.subject_to(yd[ 0]== Vo  ) # start speed
opti.subject_to(yd[-1]==-Vo  ) # end   w start speed
opti.subject_to( F[ 0]== 0   ) # start w zero leg force
opti.subject_to( F[-1]== 0   ) # end   w zero leg force
opti.subject_to(Tc    == T-Tf) # enforce task frequency


# ---- misc. constraints  ----------


# ---- initial values for solver ---
#opti.set_initial(yd, 1)
#opti.set_initial(Tc, 1)


# ---- solve NLP -------------------
opti.solver("ipopt") # set numerical backend
sol = opti.solve()   # actual solve


# ---- construct time vector --------------
tq= np.linspace(0,sol.value(T),N+1) # time vector for states
tu= np.linspace(0,sol.value(T),N  ) # time vector for controls


# ---- post-processing ------------
from pylab import plot, step, figure, legend, show, spy

plot(tq,sol.value(y ),label="y(t)")
plot(tq,sol.value(yd),label="yd(t)")
legend(loc="upper left")
figure()
plot(tq,sol.value(F ),label="F(t)")
legend(loc="upper left")
figure()
plot(tu,sol.value(Fd),label="Fd(t)")
legend(loc="upper left")


#figure()
#spy(sol.value(jacobian(opti.g,opti.x)))
#figure()
#spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))

show()
