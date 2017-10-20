import numpy as np

n = 3  # number of uavs
m = 2  # number of dimension

## Graph Theory
# Adjacency Matrix
AdjM = np.ones((n, n)) - np.eye(n);

# Degree Matrix
DegM = (n - 1) * np.eye(n);

# Laplacian matrix
L = DegM - AdjM;

# gain
kap1 = 1
kap2 = 1
kap3 = 1

## Measurement Matrix
Hx = np.concatenate((np.eye(m), 0*np.eye(m)), axis=1)
Hv = np.concatenate((0*np.eye(m), np.eye(m)), axis=1)
Hxv = np.concatenate((np.kron(np.eye(n), Hx), np.kron(np.eye(n), Hv)), axis=0)
# Hxv = [np.kron(np.eye(n), Hx),
#        np.kron(np.eye(n), Hv)]

V = 0
th = 0
Xd = np.array([0, 0, V * np.cos(th), V * np.sin(th),
      -10 * np.cos(30 - th), 10 * np.sin(30 - th), V * np.cos(th), V * np.sin(th),
      -10 * np.cos(30 + th), - 10 * np.sin(30 + th), V * np.cos(th), V * np.sin(th)])

Xd = np.array([0,0,0,0, 10,1,0,0, -10,1,0,0])

# tempt
X = np.array([0, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, 0, 0, 0.0, 0.0])

temp = np.concatenate((kap1*L, kap2*L + kap3*np.eye(n)), axis=1)
U = np.dot(np.dot(-np.kron(temp, np.eye(m)), Hxv), np.transpose(X-Xd))
print(U[1])