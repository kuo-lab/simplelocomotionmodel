# Point-mass Runner
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP

import sys
import numpy as np
from casadi import *
from random import random
from pylab import plot, figure, legend, show


N = 150 # number of control intervals
opti = Opti() # optimization problem

#%% ---- problem parameters ---------
# non-dim params
M     = 80.00  # [kg  ] body mass
g     =  9.81  # [m/s2] gravitational acc
L     =  1.00  # [m   ] leg length

# task params
v_dim =  3.00  # [m/s ] speed
f_dim =  3.00  # [Hz  ] step frequency

# tunable params
cc    = [1/0.25, 1/1.20, 5e-3] # [J] cost coefficients -> [W+, W-, |Fdd|]

# calculated params
d_dim = v_dim/f_dim  # [m ] step length
tnd   = sqrt(L/g)    # [s   ] time non-dimensional factor

# model symb params
f = opti.parameter(); opti.set_value(f ,  f_dim*tnd  ) # step frequency
d = opti.parameter(); opti.set_value(d ,  d_dim/L    ) # step length
T = 1/f  # [nd] total time duration


# ---- decision variables ---------
q = opti.variable(6,N+1) # state trajectory
u = opti.variable(5,N  ) # control var
Tc= opti.variable()      # final time
xf= opti.variable()      # foot contact position (hor)
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

Ll = sqrt((xb-xf)**2+yb**2)  # leg length
Lld= ((xb-xf)*xbd+yb*ybd)/Ll      # leg velocity
cTh= (xb-xf)/Ll # cos(Th) where Th is + ccw from -->
sTh= (yb   )/Ll # sin(Th) where Th is + ccw from -->

# energetics
P= F*Lld # leg power


dt = Tc/N # length of a control interval


#%% ---- dynamic constraints --------
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




#%% ---- Integrate for Objective Function --------
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
opti.minimize(cc[0]*Cum_pP + cc[1]*Cum_qP + cc[2]*Cum_Fdd + Cum_comp) # minimize objective function


#%% ============================================================== Constraints
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


Tf= ybd[-1]-ybd[0] # non-dim flight time (expression is divided by g, where g=1)

# ---- boundary conditions --------
opti.subject_to(xb[ 0]== 0) # start at pos 0
opti.subject_to( F[ 0]== 0) # start w zero leg force
opti.subject_to( F[-1]== 0) # end   w zero leg force
# opti.subject_to(Tc == T-(ybd[-1] - ybd[0])) # enforce task frequency
# opti.subject_to( d == xb[-1] + xbd[-1]*(ybd[-1] - ybd[0])) # body travels step length
opti.subject_to(Tc == T-Tf ) # enforce task frequency
opti.subject_to( d == xb[-1] + xbd[-1]*Tf) # body travels step length

# ---- continuity conditions --------
opti.subject_to(xbd[0]==xbd[-1])
opti.subject_to( yb[0]== yb[-1] + ybd[-1]*Tf - 0.5*Tf**2)

# ---- misc. constraints  ----------
opti.subject_to(Tc<=T) # time of contact <= step time


#%% ---- initial values for solver ---
opti.set_initial(xb , random())
opti.set_initial(xbd, random())
opti.set_initial(yb , random())
opti.set_initial(ybd, random())
# opti.set_initial(F  , random())
# opti.set_initial(Fd , random())
# opti.set_initial(Fdd, random())
# opti.set_initial(Tc , random())


#%% ---- solve NLP -------------------
opti.solver("ipopt") # set numerical backend
sol = opti.solve()   # actual solve


#%% ---- outputs ----------------------------
# ---- construct time vector --------------
tq= np.linspace(0,sol.value(Tc),N+1) # time vector for states
tu= np.linspace(0,sol.value(Tc)-sol.value(Tc)/N,N) # time vector for controls

# ---- define outputs --------------
xb = sol.value(xb);   xbd= sol.value(xbd)
yb = sol.value(yb);   ybd= sol.value(ybd)
F  = sol.value(F );   Fd = sol.value(Fd );   Fdd= sol.value(Fdd)
Tc = sol.value(Tc);   Tf = sol.value(Tf );   T  = sol.value(T  )
P  = sol.value(P );   xf = sol.value(xf )

# ---- extrapolate through flight phase --------------
Nf= int(round(Tf/sol.value(dt),0)) # number of flight pts
tf= np.linspace(0,Tf,Nf)           # time vector during flight
xbf =  xb[-1] + xbd[-1]*tf
xbdf= xbd[-1]*np.ones(Nf)
ybf =  yb[-1] + ybd[-1]*tf - 0.5*tf**2
ybdf= ybd[-1] - tf
Ff  =  0*np.zeros(Nf);   Fdf =  0*np.zeros(Nf)
Fddf=  0*np.zeros(Nf);   Pf  =  0*np.zeros(Nf)

# ---- concatenate contact and flight phases --------------
tq = vertcat(tq,tf[1:]+Tc);    tu = vertcat(tu,tf[1:]+Tc)
xb = vertcat(xb , xbf[1:]);   xbd = vertcat(xbd,xbdf[1:])
yb = vertcat(yb , ybf[1:]);   ybd = vertcat(ybd,ybdf[1:])
F  = vertcat(F  ,  Ff[1:]);    Fd = vertcat( Fd, Fdf[1:])
Fdd= vertcat(Fdd,Fddf[1:]);     P = vertcat(  P,  Pf[1:])




#%% ==================================================================== Plots
# kinematics
figure()
plot(tq,xb ,label="xb(t)")
plot(tq,xbd,label="xbd(t)")
legend(loc="upper left")
plot(tq,yb ,label="yb(t)")
plot(tq,ybd,label="ybd(t)")
legend(loc="upper left")

# Force/controls
figure()
plot(tq,F,label="F(t)")
legend(loc="upper left")
figure()
plot(tq,Fd,label="Fd(t)")
legend(loc="upper left")
figure()
plot(tu,Fdd,label="Fdd(t)")
legend(loc="upper left")

# # Slack vars (check)
# figure()
# plot(tq, P,label="P(t)")
# plot(tu, sol.value(pP),label="P(+)")
# plot(tu,-sol.value(qP),label="P(-)")
# legend(loc="upper left")



#figure()
#spy(sol.value(jacobian(opti.g,opti.x)))
#figure()
#spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))

show()
