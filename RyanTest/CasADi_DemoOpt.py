from sys import path
path.append(r"C:\Users\ryans\Dropbox\Ryan\Academia\Research\Modeling (ongoing)\simplelocomotionmodel\casadi-windows-py38-v3.5.5-64bit") #add path

from casadi import *

opti = casadi.Opti()
x = opti.variable()
y = opti.variable()

opti.minimize(   (y-x**2)**2  )
opti.subject_to( x**2+y**2==1 )
opti.subject_to(       x+y>=1 )

opti.solver('ipopt')


sol = opti.solve()

print(sol.value(x))
print(sol.value(y))