""" Python implementation of
    Algorithm for Automatically Fitting Digitized Curves
    by Philip J. Schneider
    "Graphics Gems", Academic Press, 1990
"""
import numpy as np
from . import cubic

import pdb

# Fit one (ore more) Bezier curves to a set of points
def fitCurve(points, maxError):
    _y = points
    _nints = _y.shape[0]-1
    _u = chordLengthParameterize( _y )
    _h = np.array( [ _u[i+1]-_u[i] for i in range(_nints) ] )

    return fitCubic(_y, _u, _h, _nints, maxError, _dim=points[0].shape[-1])

def _abcd(_y, _h, _d2y):
    _a = _y[:len(_h)]
    _b = np.array( [ (_y[i+1]-_y[i])/_h[i] - (2.*_d2y[i]+_d2y[i+1])*_h[i]/6. for i in range(len(_h)) ] )
    _c = _d2y[:len(_h)]*0.5
    _d = np.array( [ (_d2y[i+1]-_d2y[i])/_h[i]/6. for i in range(len(_h)) ] )
    return (_a, _b, _c, _d)

def natural(_y, _h, _d2y0=0.0, _d2yn=0.0, _dim=1):
    _n = len(_h)
    _A = np.zeros((_n-1, _n-1))
    _b = np.zeros((_n-1,))
    for ix in range(_n-1):
        _A[ix,ix] = 2*(_h[ix]+_h[ix+1])
        if ix>0:    _A[ix,ix-1] = _h[ix]
        if ix<_n-2: _A[ix,ix+1] = _h[ix+1]
        _b[ix] = 6*( (_y[ix+2]-_y[ix+1])/_h[ix+1] - (_y[ix+1]-_y[ix])/_h[ix] )
    _b[0]  -= _h[0]  *_d2y0
    _b[-1] -= _h[_n-1]*_d2yn
    _z = np.linalg.solve(_A, _b)

    _d2y = np.insert(_z,(0,len(_z)),(_d2y0,_d2yn))
    return _abcd(_y, _h, _d2y)

# clamped spline boundary conditions (guessed from data)
def clamped(_y, _u, _h, _dim=1):
    _n = len(_h)
    _dy0 = (_y[1]-_y[0])/(_u[1]-_u[0])
    _dyn = (_y[-1]-_y[-2])/(_u[-1]-_u[-2])

    _A = np.zeros((_n-1, _n-1))
    _b = np.zeros((_n-1,))

    _A[0,:2]   = 1.5*_h[0]+2.*_h[1], _h[1]
    _A[-1,-2:] = _h[_n-2],            2.*_h[_n-2]+1.5*_h[_n-1]
    _b[0] = 6.*( (_y[2]-_y[1]) /_h[1] - (_y[1]-_y[0])  /_h[0] ) - 3.*( (_y[1]-_y[0])/_h[0] - _dy0)
    _b[-1] = 6.*( (_y[_n]-_y[_n-1])/_h[_n-1] - (_y[_n-1]-_y[_n-2])/_h[_n-2] ) - 3.*( _dyn - (_y[_n]-_y[_n-1])/_h[_n-1] )

    for ix in range(1,_n-2):
        _A[ix,ix-1:ix+2] = _h[ix], 2.*(_h[ix]+_h[ix+1]), _h[ix+1]
        _b[ix] = 6.*( (_y[ix+2]-_y[ix+1])/_h[ix+1] - (_y[ix+1]-_y[ix])/_h[ix] )
    _z = np.linalg.solve(_A, _b)

    _d2y0 = 3./_h[0]*  ( (_y[1]-_y[0])/_h[0] - _dy0 ) - 0.5*_z[0]
    _d2yn = 3./_h[_n-1]*( _dyn - (_y[_n]-_y[_n-1])/_h[_n-1] )
    _d2y = np.insert(_z,(0,len(_z)),(_d2y0,_d2yn))
    return _abcd(_y, _h, _d2y)

def notaknot(_y, _h, _dim=1):
    _n = len(_h)
    _A = np.zeros((_n-1, _n-1))
    _b = np.zeros((_n-1,))

    _A[0,:2]   = 3.*_h[0]+2.*_h[1]+_h[0]*_h[0]/_h[1], _h[1]-_h[0]*_h[0]/_h[1]
    _A[-1,-2:] = _h[_n-2]-_h[_n-1]*_h[_n-1]/_h[_n-2],     2.*_h[_n-2]+3.*_h[_n-1]+_h[_n-1]*_h[_n-1]/_h[_n-2]
    for i in range(_n-1):
        if i>0 and i<_n-2:
            _A[i,i-1:i+2] = _h[i], 2.*(_h[i]+_h[i+1]), _h[i+1]
        _b[i] = 6.*( (_y[i+2]-_y[i+1])/_h[i+1] - (_y[i+1]-_y[i])/_h[i] )
    _z = np.linalg.solve(_A,_b)

    _d2y0 = _z[0]  - _h[0]/_h[1]*    ( _z[1]-_z[0] )
    _d2yn = _z[-1] + _h[_n-1]/_h[_n-2]*( _z[-1]-_z[-2] )
    _d2y = np.insert(_z,(0,len(_z)),(_d2y0,_d2yn))
    return _abcd(_y, _h, _d2y)

def fitCubic(_y, _u, _h, _nints, error, _dim=2):
    points = _y

    bezCurve = generateBezier(points, u, leftTangent, rightTangent, _dim=_dim)
    # Find max deviation of points to fitted curve
    maxError, splitPoint = computeMaxError(points, bezCurve, u)
    if maxError < error:
        return [bezCurve]

    # If error not too large, try some reparameterization and iteration
    if maxError < error**2:
        for _ in range(20):
            uPrime = reparameterize(bezCurve, points, u)
            bezCurve = generateBezier(points, uPrime, leftTangent, rightTangent, _dim=_dim)
            maxError, splitPoint = computeMaxError(points, bezCurve, uPrime)
            if maxError < error:
                return [bezCurve]
            u = uPrime

    # Fitting failed -- split at max error point and fit recursively
    beziers = []
    centerTangent = normalize(points[splitPoint-1] - points[splitPoint+1])
    beziers += fitCubic(points[:splitPoint+1], leftTangent, centerTangent, error, _dim=_dim)
    beziers += fitCubic(points[splitPoint:], -centerTangent, rightTangent, error, _dim=_dim)

    return beziers


def generateBezier(points, parameters, leftTangent, rightTangent, _dim=2):
    bezCurve = [points[0], None, None, points[-1]]
    # compute the A's
    A = np.zeros((len(parameters), 2, _dim))
    for i, u in enumerate(parameters):
        A[i][0] = leftTangent  * 3*(1-u)**2 * u
        A[i][1] = rightTangent * 3*(1-u)    * u**2

    # Create the C and X matrices
    C = np.zeros((2, 2))
    X = np.zeros(2)

    for i, (point, u) in enumerate(zip(points, parameters)):
        C[0][0] += np.dot(A[i][0], A[i][0])
        C[0][1] += np.dot(A[i][0], A[i][1])
        C[1][0] += np.dot(A[i][0], A[i][1])
        C[1][1] += np.dot(A[i][1], A[i][1])

        tmp = point - bezier.q([points[0], points[0], points[-1], points[-1]], u)

        X[0] += np.dot(A[i][0], tmp)
        X[1] += np.dot(A[i][1], tmp)

    # Compute the determinants of C and X
    det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1]
    det_C0_X  = C[0][0] * X[1] - C[1][0] * X[0]
    det_X_C1  = X[0] * C[1][1] - X[1] * C[0][1]

    # Finally, derive alpha values
    alpha_l = 0.0 if det_C0_C1 == 0 else det_X_C1 / det_C0_C1
    alpha_r = 0.0 if det_C0_C1 == 0 else det_C0_X / det_C0_C1

    # If alpha negative, use the Wu/Barsky heuristic (see text) */
    # (if alpha is 0, you get coincident control points that lead to
    # divide by zero in any subsequent NewtonRaphsonRootFind() call. */
    segLength = np.linalg.norm(points[0] - points[-1])
    epsilon = 1.0e-6 * segLength
    if alpha_l < epsilon or alpha_r < epsilon:
        # fall back on standard (probably inaccurate) formula, and subdivide further if needed.
        bezCurve[1] = bezCurve[0] + leftTangent * (segLength / 3.0)
        bezCurve[2] = bezCurve[3] + rightTangent * (segLength / 3.0)

    else:
        # First and last control points of the Bezier curve are
        # positioned exactly at the first and last data points
        # Control points 1 and 2 are positioned an alpha distance out
        # on the tangent vectors, left and right, respectively
        bezCurve[1] = bezCurve[0] + leftTangent * alpha_l
        bezCurve[2] = bezCurve[3] + rightTangent * alpha_r

    return bezCurve


def reparameterize(bezier, points, parameters):
    return [newtonRaphsonRootFind(bezier, point, u) for point, u in zip(points, parameters)]


def newtonRaphsonRootFind(bez, point, u):
    """
       Newton's root finding algorithm calculates f(x)=0 by reiterating
       x_n+1 = x_n - f(x_n)/f'(x_n)

       We are trying to find curve parameter u for some point p that minimizes
       the distance from that point to the curve. Distance point to curve is d=q(u)-p.
       At minimum distance the point is perpendicular to the curve.
       We are solving
       f = q(u)-p * q'(u) = 0
       with
       f' = q'(u) * q'(u) + q(u)-p * q''(u)

       gives
       u_n+1 = u_n - |q(u_n)-p * q'(u_n)| / |q'(u_n)**2 + q(u_n)-p * q''(u_n)|
    """
    d = cubic.q(bez, u)-point
    numerator = (d * cubic.qprime(bez, u)).sum()
    denominator = (cubic.qprime(bez, u)**2 + d * cubic.qprimeprime(bez, u)).sum()

    if denominator == 0.0:
        return u
    else:
        return u - numerator/denominator


def chordLengthParameterize(points):
    u = [0.0]
    for i in range(1, len(points)):
        u.append(u[i-1] + np.linalg.norm(points[i] - points[i-1]))

    for i, _ in enumerate(u):
        u[i] = u[i] / u[-1]

    return u


def computeMaxError(points, bez, parameters):
    maxDist = 0.0
    splitPoint = len(points)/2
    for i, (point, u) in enumerate(zip(points, parameters)):
        dist = np.linalg.norm(cubic.q(bez, u)-point)**2
        if dist > maxDist:
            maxDist = dist
            splitPoint = i

    return maxDist, splitPoint


def normalize(v):
    return v / np.linalg.norm(v)

