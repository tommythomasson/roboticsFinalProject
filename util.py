import numpy as np

def dh_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta),  -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha),  a*np.cos(theta)],
        [np.sin(theta),   np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha),  a*np.sin(theta)],
        [ 0,              np.sin(alpha),                np.cos(alpha),                d],
        [ 0,              0,                            0,                            1]
    ])

# Define all helper functions here:
def forward_kinematics(dh_table):
    T = np.eye(4) # used to not need an if check for table size 1: ID * T = T
    origins = [T[:3, 3]]
    z_axes = [T[:3, 2]]
    for params in dh_table:
        Ti = dh_matrix(*params) # * decomposes the list into arguments
        T = T @ Ti # @ is for matrix multiplication
        origins.append(T[:3, 3])
        z_axes.append(T[:3, 2])
    return T, origins, z_axes

# if we decide that we need to do jacobians... but probably will not need them
def jacobian(dh_table):
    n = len(dh_table)
    
    _, origins, z_axes = forward_kinematics(dh_table)
    
    o_final = origins[-1]
    
    Jv_linear = np.zeros((3, n))
    Jw_angular = np.zeros((3, n))
    
    for i in range(n):
        z = z_axes[i]
        o = origins[i]
        
        Jv_linear[:, i] = np.cross(z, (o_final - o))
        Jw_angular[:, i] = z
    
    J = np.vstack((Jv_linear, Jw_angular))
    return J

# inverse kinematics for a RPP manipulator are straight forward...
# there do not need to be any checks done for gimbal lock or other phenomena
def inverse_kinematics(x, y, z):
    theta = np.arctan2(y, x)
    d_arm2 = np.sqrt(x**2 + y**2) # or - offset, but this distance is likely to be relative to the origin.
    d_arm3 = z # or robot height - z
    return theta, d_arm2, d_arm3