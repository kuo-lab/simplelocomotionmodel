# Monopedal hopper
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP
from sys import path
path.append(r"C:\Users\ryans\Dropbox\Ryan\Academia\Research\Modeling (ongoing)\simplelocomotionmodel\casadi-windows-py38-v3.5.5-64bit") #add path

from casadi import *

N = 100 # number of control intervals

opti = Opti() # Optimization problem

# ---- decision variables ---------
q = opti.variable(2,N+1) # state trajectory
y = q[0,:] # vrt pos
yd= q[1,:] # vrt vel
u = opti.variable(1,N)   # leg force
T = opti.variable()      # final time



# ---- dynamic constraints --------
f = lambda q,u: vertcat(q[1],u-1) # dx/dt = f(x,u)

dt = T/N # length of a control interval
for k in range(N): # loop over control intervals
   # Runge-Kutta 4 integration
   k1 = f(q[:,k],         u[:,k])
   k2 = f(q[:,k]+dt/2*k1, u[:,k])
   k3 = f(q[:,k]+dt/2*k2, u[:,k])
   k4 = f(q[:,k]+dt*k3,   u[:,k])
   x_next = q[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
   opti.subject_to(q[:,k+1]==x_next) # close the gaps


# ---- objective function --------
f2 = lambda u: u**2 # integrand = u^2
for k in range(N): # loop over control intervals
   # Runge-Kutta 4 integration of objective function
   k1 = f2(u[:,k]        )        
   k2 = f2(u[:,k]+dt/2*k1)
   k3 = f2(u[:,k]+dt/2*k2)
   k4 = f2(u[:,k]+dt*k3, )
   x_next = k1 + dt/6*(k1+2*k2+2*k3+k4)

#test this now


# ---- objective          ---------
opti.minimize(x_next) # force squared


# ---- path constraints -----------
opti.subject_to(u>=0)   # extension force only
opti.subject_to(y>=0)   # body above ground

# ---- boundary conditions --------
opti.subject_to( y[ 0]==1)      # start w straight leg ...
opti.subject_to( y[-1]==1)      # end   w straight leg ...
opti.subject_to(yd[-1]==-yd[0]) # end   w start speed  ...
#opti.subject_to(speed[0]==0) # ... from stand-still 



# ---- misc. constraints  ----------
opti.subject_to(T>=0) # Time must be positive

# ---- initial values for solver ---
#opti.set_initial(yd, 1)
#opti.set_initial( T, 1)

# ---- solve NLP              ------
opti.solver("ipopt") # set numerical backend
sol = opti.solve()   # actual solve

# ---- post-processing        ------
from pylab import plot, step, figure, legend, show, spy

plot(sol.value(yd),label="vrt vel")
plot(sol.value(y ),label="vrt pos")
legend(loc="upper left")
#plot(limit(sol.value(y)),'r--',label="speed limit")
figure()
step(range(N),sol.value(u),'k',label="leg force")
legend(loc="upper left")

#figure()
#spy(sol.value(jacobian(opti.g,opti.x)))
#figure()
#spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))

show()
