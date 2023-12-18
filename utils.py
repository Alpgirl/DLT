import numpy as np

import pytransform3d.rotations as pr
from pytransform3d.plot_utils import plot_vector

# constants

origin = np.array([0, 0, 0])

# basis vectors
x = np.array([1, 0, 0])
y = np.array([0, 1, 0])
z = np.array([0, 0, 1])

# basis vectors as homogeneous coordinates
xh = np.array([1, 0, 0, 1])
yh = np.array([0, 1, 0, 1])
zh = np.array([0, 0, 1, 1])

# helper functions
def get_rot_x(angle):
    '''
    transformation matrix that rotates a point about the standard X axis
    '''
    Rx = np.zeros(shape=(3, 3))
    Rx[0, 0] = 1
    Rx[1, 1] = np.cos(angle)
    Rx[1, 2] = -np.sin(angle)
    Rx[2, 1] = np.sin(angle)
    Rx[2, 2] = np.cos(angle)
    
    return Rx

def get_rot_y(angle):
    '''
    transformation matrix that rotates a point about the standard Y axis
    '''
    Ry = np.zeros(shape=(3, 3))
    Ry[0, 0] = np.cos(angle)
    Ry[0, 2] = -np.sin(angle)
    Ry[2, 0] = np.sin(angle)
    Ry[2, 2] = np.cos(angle)
    Ry[1, 1] = 1
    
    return Ry

def get_rot_z(angle):
    '''
    transformation matrix that rotates a point about the standard Z axis
    '''
    Rz = np.zeros(shape=(3, 3))
    Rz[0, 0] = np.cos(angle)
    Rz[0, 1] = -np.sin(angle)
    Rz[1, 0] = np.sin(angle)
    Rz[1, 1] = np.cos(angle)
    Rz[2, 2] = 1
    
    return Rz

def create_rotation_transformation_matrix(angles, order):
    '''
    Create a matrix that rotates a vector through the given angles in the given order
    wrt the standard global axes (extrinsic rotation)
    Note: The rotation is carried out anti-clockwise in a left handed axial system
    
    Parameters
    -----------
    angles - list
        list of angles in radians
    order - string
        the order in which to rotate
        
    Returns
    --------
    net - np.ndarray, shape - (3, 3)
        The transformation matrix which carries out the given rotations
    '''
    fn_mapping = {'x': get_rot_x, 'y': get_rot_y, 'z': get_rot_z}
    net = np.identity(3)
    for angle, axis in list(zip(angles, order))[::-1]:
        if fn_mapping.get(axis) is None:
            raise ValueError("Invalid axis")
        R = fn_mapping.get(axis)
        net = np.matmul(net, R(angle))
        
    return net

def create_translation_matrix(offset):
    '''
    Create a transformation matrix that translates a vetor by the given offset
    
    Parameters
    -----------
    offset - np.ndarray, shape - (3,)
        The translation offset
    
    Returns
    ----------
    T - np.ndarray, shape - (4, 4)
        The translation matrix
    '''
    T = np.identity(4)
    T[:3, 3] = offset
    return T

make_line = lambda u, v: np.vstack((u, v)).T

def create_image_grid(f, img_size):
    '''
    Create an image grid of the given size parallel to the XY plane
    at a distance f from the camera center (origin)
    '''
    h, w = img_size
    xx, yy = np.meshgrid(range(-(h // 2), w // 2 + 1), range(-(h // 2), w // 2 + 1))
    Z = np.ones(shape=img_size) * f

    return xx, yy, Z

def convert_grid_to_homogeneous(xx, yy, Z, img_size):
    '''
    Extract coordinates from a grid and convert them to homogeneous coordinates
    '''
    h, w = img_size
    pi = np.ones(shape=(4, h*w))
    c = 0
    for i in range(h):
        for j in range(w):
            x = xx[i, j]
            y = yy[i, j]
            z = Z[i, j]
            point = np.array([x, y, z])
            pi[:3, c] = point
            c += 1
    return pi

def convert_homogeneous_to_grid(pts, img_size):
    '''
    Convert a set of homogeneous points to a grid
    '''
    xxt = pts[0, :].reshape(img_size)
    yyt = pts[1, :].reshape(img_size)
    Zt = pts[2, :].reshape(img_size)

    return xxt, yyt, Zt
    
def compute_intrinsic_parameter_matrix(f, s, a, cx, cy):
    K = np.identity(3)
    K[0, 0] = f
    K[0, 1] = s
    K[0, 2] = cx
    K[1, 1] = a * f
    K[1, 2] = cy
    
    return K
    
def compute_image_projection(points, K):
    '''
    Compute projection of points onto the image plane
    
    Parameters
    -----------
    points - np.ndarray, shape - (3, n_points)
        points we want to project onto the image plane
        the points should be represented in the camera coordinate system
    K - np.ndarray, shape - (3, 3)
        camera intrinsic matrix
        
    Returns
    -------
    points_i - np.ndarray, shape - (2, n_points)
        the projected points on the image
    '''
        
    h_points_i = K @ points
    
    h_points_i[0, :] = h_points_i[0, :] / h_points_i[2, :]
    h_points_i[1, :] = h_points_i[1, :] / h_points_i[2, :]

    points_i = h_points_i[:2, :]    
    
    return points_i
    
def generate_random_points(n_points, xlim, ylim, zlim):
    '''
    Generate random points in the given limits
    '''
    x = np.random.randint(xlim[0], xlim[1], size=n_points)
    y = np.random.randint(ylim[0], ylim[1], size=n_points)
    z = np.random.randint(zlim[0], zlim[1], size=n_points)
    
    return np.vstack((x, y, z))
    
def compute_coordinates_wrt_camera(world_points, E, is_homogeneous=False):
    '''
    Performs a change of basis operation from the world coordinate system
    to the camera coordinate system
    
    Parameters
    ------------
    world_points - np.ndarray, shape - (3, n_points) or (4, n_points)
             points in the world coordinate system
    E - np.ndarray, shape - (3, 4)
        the camera extrinsic matrix
    is_homogeneous - boolean
        whether the coordinates are represented in their homogeneous form
        if False, an extra dimension will  be added for computation
        
    Returns
    ----------
    points_c - np.ndarray, shape - (3, n_points)
             points in the camera coordinate system
    '''
    if not is_homogeneous:
        # convert to homogeneous coordinates
        points_h = np.vstack((world_points, np.ones(world_points.shape[1])))
        
    points_c = E @ points_h
    return points_c