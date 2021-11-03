from sys import path
path.append(r"C:\Users\ryans\Dropbox\Ryan\Academia\Research\Modeling (ongoing)\simplelocomotionmodel\casadi-windows-py38-v3.5.5-64bit") #add path

from casadi import *
x = MX.sym("x")           #create arbitrary var
#print(jacobian(sin(x),x)) #test CasADi derivative


# SX symbolics type - intended for low-level operations, sequence of scalar operations
y = SX.sym('y',5)
z = SX.sym('z',4,2)


f = x**2 + 10
f = sqrt(f)
#print(f)


# DM symbolics type - similar to SX, but nonzero elements are numerical, not symbolic
C = DM(2,3)

C_dense = C.full()
from numpy import array
C_dense = array(C) #equivalent

# MX symbolics type - MX acts as "glue" and enables formulation of constraint function of NLP (calling ODE/DAE integrators, or if expressions are too large)
x = SX.sym('x',2,2)
y = SX.sym('y')
f = 3*x+y
#print(f)
#print(f.shape)


x = MX.sym('x',2,2)
y = MX.sym('y')
f = 3*x + y
#print(f)
#print(f.shape)

#print(x[0,0])


x = MX.sym('x',2)
A = MX(2,2)
A[0,0] = x[0]
A[1,1] = x[0]+x[1]
#print('A:',A)


# Sparsity class
#print(SX.sym('x',Sparsity.lower(3)))


# Getting/setting elements in matrices
M = SX([[3,7],[4,5]])
#print(M[0,:])

M[0,:] = 1
#print(M)


M = SX([[3,7],[4,5]])
M[0,:][0,0] = 1
#print(M)

M = diag(SX([3,4,5,6]))
#print(M)

#print(M[ 0, 0])
#print(M[ 1, 0])
#print(M[-1,-1])


#print(M[:,1])
#print(M[1:,1:4:2])

M = SX([[3,7,8,9],[4,5,6,1]])
#print(M)
#print(M[0,[0,3]], M[[5,-6]])


# Arithmetic operations
x = SX.sym('x')
y = SX.sym('y',2,2)
#print(sin(y)-x)

#print(y*y,y@y) #print y^2 and y*y as matrix multiplication

#print(y)
#print(y.T) #transposed y

x = SX.eye(4)
#print(reshape(x,2,8)) #similar to MATLAB reshape


#Concatenation
x = SX.sym('x',5)
y = SX.sym('y',5)

#print(vertcat(x,y))
#print(horzcat(x,y))