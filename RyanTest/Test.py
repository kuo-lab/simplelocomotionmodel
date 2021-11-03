from sys import path
path.append(r"C:\Users\ryans\Dropbox\Ryan\Academia\Research\Modeling (ongoing)\simplelocomotionmodel\casadi-windows-py38-v3.5.5-64bit") #add path

from casadi import *
x = MX.sym("x")           #create arbitrary var
print(jacobian(sin(x),x)) #test CasADi derivative


# SX symbolics type
y = SX.sym('y',5)
z = SX.sym('z',4,2)


f = x**2 + 10
f = sqrt(f)
print(f)


# DM symbolics type
C = DM(2,3)

C_dense = C.full()
from numpy import array
C_dense = array(C) #equivalent

# MX symbolics type
x = SX.sym('x',2,2)
y = SX.sym('y')
f = 3*x+y
print(f)
print(f.shape)


x = MX.sym('x',2,2)
y = MX.sym('y')
f = 3*x + y
print(f)
print(f.shape)

print(x[0,0])


x = MX.sym('x',2)
A = MX(2,2)
A[0,0] = x[0]
A[1,1] = x[0]+x[1]
print('A:',A)