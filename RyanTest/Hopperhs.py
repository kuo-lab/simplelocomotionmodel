# Point-mass Hopper
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# Uses Hermite Simpson
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

f = opti.parameter(); opti.set_value(f ,  f_dim*tnd  ) # frequency
Vo= opti.parameter(); opti.set_value(Vo, Vo_dim*tnd/L) # speed at initial contact
T = 1/f        # [nd] total time duration
Tf= Tf_dim/tnd # [nd] flight time


# ---- decision variables ---------
q = opti.variable(4,N+1) # state trajectory
u = opti.variable(5,N+1) # control var
Tc= opti.variable()      # final time
y   = q[0,:] # vrt pos
yd  = q[1,:] # vrt vel
F   = q[2,:] # leg force
Fd  = q[3,:] # leg force rate
Fdd = u[0,:] # leg force rate2
pP  = u[1,:] # pos leg power
qP  = u[2,:] # neg leg power
pFdd= u[3,:] # pos force rate2
qFdd= u[4,:] # neg force rate2


dt = Tc/N # length of a control interval


# ---- dynamic constraints --------
# ----------------------(   yd,    ydd,   Fd,  Fdd)
qd = lambda q,u: vertcat( q[1], q[2]-1, q[3], u[0]) # dq/dt = f(q,u)

for k in range(N): # loop over control intervals
   # Hermite Simpson quadrature
   f = qd(q[:,k], u[:,k])
   fnext = qd(q[:,k+1], u[:,k+1])
   qhalf = 0.5*(q[:,k] + q[:,k+1]) + dt/8*(f - fnext)
   uhalf = 0.5*(u[:,k] + u[:,k+1])
   fhalf = qd(qhalf, uhalf)
   opti.subject_to(q[:,k+1] - q[:,k] == dt/6*(f + 4*fhalf + fnext)) # close the gaps

# add an additional constraint to force u[N]
opti.subject_to(u[:,N]==0.)

P= F*yd # leg power

# ---- Integrate for Objective Function --------
Int = lambda pP,qP,pFdd,qFdd:  pP+qP + pFdd+qFdd + pP*qP + pFdd*qFdd # J = |power| + |Fdd| & complementarity
cumInt= 0
for k in range(1,N): # loop over control intervals
   cumInt = cumInt + dt*( Int(pP[:,k-1],qP[:,k-1],pFdd[:,k-1],qFdd[:,k-1]) + \
                          Int(pP[:,k  ],qP[:,k  ],pFdd[:,k  ],qFdd[:,k  ]) )/2

# ---- objective ------------------
opti.minimize(cumInt) # minimize objective function


# ---- path constraints -----------
opti.subject_to(   y>=0)  # body above ground
opti.subject_to(  pP>=0)  # bounded control
opti.subject_to(  qP>=0)  # bounded control
opti.subject_to(pFdd>=0)  # bounded control
opti.subject_to(qFdd>=0)  # bounded control
opti.subject_to(   F>=0)  # bounded control
opti.subject_to(P[0:N+1] ==pP  -qP  )  # power slack vars
opti.subject_to(  Fdd  ==pFdd-qFdd)  # Fdd   slack vars


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
tq= np.linspace(0,sol.value(Tc),N+1) # time vector for states
tu = np.linspace(0, sol.value(Tc),N+1) # time vector for controls


# ---- post-processing ------------
from pylab import plot, step, figure, legend, show, spy

plot(tq,sol.value(y ),label="y(t)")
plot(tq,sol.value(yd),label="yd(t)")
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
