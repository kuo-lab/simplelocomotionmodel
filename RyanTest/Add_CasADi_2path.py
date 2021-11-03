from sys import path
path.append(r"C:\Users\ryans\Dropbox\Ryan\Academia\Research\Modeling (ongoing)\simplelocomotionmodel\casadi-windows-py38-v3.5.5-64bit") #add path

from casadi import *
x = MX.sym("x")           #create arbitrary var
print(jacobian(sin(x),x)) #test CasADi derivative


