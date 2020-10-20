import numpy as np


# evaluates cubic bezier at x, return point
# tcPoly --> (xk, a, b, c, d)
def q(tcPoly, x):
    # q = a[i]+dx*(b[i]+dx*(c[i]+dx*d[i]))
    dx = x-tcPoly[0] 
    return tcPoly[1] + dx*(tcPoly[2] + dx*(tcPoly[3] + dx*tcPoly[4]))


# evaluates cubic bezier first derivative at x, return point
# tcPoly --> (xk, a, b, c, d)
def qprime(tcPoly, x):
    # dq = b[i]+dx*(2.*c[i]+dx*3.*d[i])
    dx = x-tcPoly[0] 
    return tcPoly[2] + dx*(2.*tcPoly[3] + dx*3.*tcPoly[4])


# evaluates cubic bezier second derivative at x, return point
# tcPoly --> (xk, a, b, c, d)
def qprimeprime(tcPoly, x):
    # ddq = 2.*c[i]+6.*d[i]*(x-xk[i])
    dx = x-tcPoly[0] 
    return 2.*tcPoly[3] + dx*6.*tcPoly[4]


# cPolyAll --> [ (xk, a, b, c, d), [..], .. n rows ]
# _tcPoly --> (xk, a, b, c, d)
def qint(cPolyAll):
    _values = [ _qintvalue(cPolyAll[i], cPolyAll[i+1, 0]-cPolyAll[i, 0]) for i in range(cPolyAll.shape[0]-1) ]
    return sum(_values)

# evaluate the integral of q at x (note there's no (x-xk) here!)
# tcPoly --> (xk, a, b, c, d)
def _qintvalue(tcPoly, x):
    return x*(tcPoly[1] + x*(0.5*tcPoly[2] + x*(1./3.*tcPoly[3] + 0.25*tcPoly[4]*x)))