import numpy as np


# evaluates cubic bezier at t, return point
def q(ctrlPoly, t):
    return (1.0-t)**3 * ctrlPoly[0] + 3*(1.0-t)**2 * t * ctrlPoly[1] + 3*(1.0-t)* t**2 * ctrlPoly[2] + t**3 * ctrlPoly[3]


# evaluates cubic bezier first derivative at t, return point
def qprime(ctrlPoly, t):
    return 3*(1.0-t)**2 * (ctrlPoly[1]-ctrlPoly[0]) + 6*(1.0-t) * t * (ctrlPoly[2]-ctrlPoly[1]) + 3*t**2 * (ctrlPoly[3]-ctrlPoly[2])


# evaluates cubic bezier second derivative at t, return point
def qprimeprime(ctrlPoly, t):
    return 6*(1.0-t) * (ctrlPoly[2]-2*ctrlPoly[1]+ctrlPoly[0]) + 6*(t) * (ctrlPoly[3]-2*ctrlPoly[2]+ctrlPoly[1])


# approx qint(0,1) = 12 (a^2 + 3 (b^2 - b c + c^2) - 3 c d + d^2 + a (-3 b + d))
# https://raphlinus.github.io/curves/2018/12/28/bezier-arclength.html
def qint_approx(ctrlPoly):
    return ( 12 *( ctrlPoly[0]**2 \
                + 3 *(ctrlPoly[1]**2 - ctrlPoly[1]*ctrlPoly[2] + ctrlPoly[2]**2)
                - 3 * ctrlPoly[2] * ctrlPoly[3] + ctrlPoly[3]**2 
                + ctrlPoly[0] * (-3 *ctrlPoly[1] + ctrlPoly[3]) ) ).sum()

def qint_multi(ctrlPolyMulti):
    _num = len(ctrlPolyMulti)
    return sum([ qint_approx(ctrlPolyMulti[_ix:_ix+4]) for  _ix in range(0, _num-1, 3) ])

    # _values = []
    # for _ix in range(0, _num, 4):
    #     _ctrlPoly = ctrlPolyMulti[_ix:_ix+4]
    #     _values.append( qint(_ctrlPoly) )
    # return sum(_values)