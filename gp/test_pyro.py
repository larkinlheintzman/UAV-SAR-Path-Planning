import torch

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

import pyro
import pyro.contrib.gp as gp
import pyro.distributions as dist

import pdb

from tqdm import tqdm, trange

ls_z = torch.tensor([3.0])

t = torch.linspace(-5, 5, 20)
tpos = torch.linspace(0, 5, 10)
x, y = torch.meshgrid(t, t)
print(x.shape, y.shape, t.shape)

X = torch.cat( ( x.flatten().view(1, -1), y.flatten().view(1, -1) ) ).transpose(1,0)

Y = torch.linspace(0, 100, 400) + torch.rand((400)) * 50 - 25

# X = np.asarray(X, dtype=float)
print(X.shape, type(X))

def ls_func3(x, ls_z):
    ls0 = ls1 = x[:, 2:]
    ls2 = ls_z * torch.ones_like(ls0)
    return torch.cat( (ls0, ls1, ls2), 1)

def ls_func(x):
    ls0 = ls1 = x[:, 1:]
    return torch.cat( (ls0, ls1), 1)

kernel = gp.kernels.gibbs.Gibbs( input_dim=2, lengthscale_fn=ls_func )

gpr = gp.models.GPRegression(X, Y, kernel)

optimizer = torch.optim.Adam(gpr.parameters(), lr=0.005)
loss_fn = pyro.infer.Trace_ELBO().differentiable_loss

losses = []
num_steps = 2
for i in trange(num_steps):
    optimizer.zero_grad()
    loss = loss_fn(gpr.model, gpr.guide)
    loss.backward()
    optimizer.step()
    losses.append(loss.item())

x_t = torch.rand((10, 2)) * 10 - 5

with  torch.no_grad():
    m, cov = gpr(x_t, full_cov=True)

print(m.shape)
print(cov.shape)

x_t_np = x_t.numpy()
m_np = m.numpy()

X_np = X.numpy()
Y_np = Y.numpy()

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot( xs=x_t_np[:,0], ys=x_t_np[:,1], zs=m_np, color='red' )
ax.scatter( xs=X_np[:,0], ys=X_np[:,1], zs=Y_np, color='blue' )

plt.show()

print(np.linalg.det(cov))
print(np.trace(cov))

print('Done!')

# (Pdb) X.shape
# torch.Size([400, 2])
# (Pdb) Z.shape
# torch.Size([10, 2])
# (Pdb) rX.shape
# torch.Size([400, 2])
# (Pdb) rZ.shape
# torch.Size([10, 2])
# (Pdb) rX2.shape
# torch.Size([400, 1, 2])
# (Pdb) rZ2.shape
# torch.Size([1, 10, 2])
# (Pdb) rX2_plus_rZ2.shape
# torch.Size([400, 10])
# (Pdb) KK = ( _torch_sqrt( torch.pow( (2.0 * rX.matmul(rZ.t())) / rX2_plus_rZ2, dim ) )* torch.exp( -1.0 * r2 / rX2_plus_rZ2 ) )
# (Pdb) KK.shape
# torch.Size([400, 10])