import numpy as np
from scipy.optimize import minimize
from typing import Optional

def normalize_points_2d(points):
    # Calculate the centroid of the points
    mean = np.mean(points, 0)
    std_dev = np.std(points)
    scale = np.sqrt(2) / std_dev

    T = np.array([
        [scale, 0, -scale * mean[0]],
        [0, scale, -scale * mean[1]],
        [0, 0, 1]
    ])

    # Apply transformation
    points_h = np.hstack((points, np.ones((points.shape[0], 1))))
    points_normalized_h = (T @ points_h.T).T

    # Ignore the homogeneous coordinate to return to 2D
    points_normalized = points_normalized_h[:, :2]

    return T, points_normalized


def normalize_points_3d(points):
    # Calculate the centroid of the points
    mean = np.mean(points, 0)
    std_dev = np.std(points)
    scale = np.sqrt(3) / std_dev

    # Create the normalization matrix
    U = np.array([
        [scale, 0,     0,     -scale * mean[0]],
        [0,     scale, 0,     -scale * mean[1]],
        [0,     0,     scale, -scale * mean[2]],
        [0,     0,     0,     1]
    ])

    # Apply transformation
    points_h = np.hstack((points, np.ones((points.shape[0], 1))))
    points_normalized_h = (U @ points_h.T).T

    # Ignore the homogeneous coordinate to return to 3D
    points_normalized = points_normalized_h[:, :3]

    return U, points_normalized


def dlt_algorithm(points_2d, points_3d):
    A = []
    for i in range(points_2d.shape[0]):
        X, Y, Z = points_3d[i]
        u, v = points_2d[i]
        A.append([X, Y, Z, 1, 0, 0, 0, 0, -u*X, -u*Y, -u*Z, -u])
        A.append([0, 0, 0, 0, X, Y, Z, 1, -v*X, -v*Y, -v*Z, -v])

    A = np.array(A)
    _, _, V = np.linalg.svd(A)
    P_normalized = V[-1].reshape((3, 4))

    return P_normalized

def reprojection_error(params, points_2d, points_3d):
    # Reshape params into 3x4 projection matrix P
    P = params.reshape((3, 4))


    # Convert 3D points to homogeneous coordinates
    points_3d_homog = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))

    # Project 3D points to 2D using P
    points_2d_proj_homog = P.dot(points_3d_homog.T).T
    points_2d_proj = points_2d_proj_homog[:, :2] / points_2d_proj_homog[:, [2]]

    # Compute the difference (reprojection error) between the
    # observed and projected 2D points
    error = np.sum((points_2d - points_2d_proj)**2, axis=1)
    error = np.sqrt(error).sum()
    return error

def project_points(P, points_3d):
    """
    Project 3D points into 2D space using the projection matrix P.

    Args:
    P - transformation matrix of shape 3 x 4
    points3d - points in world space with shape n x 3, where n - number of points

    return points projected onto image space with shape n x 2
    """

    # Homogenize the 3D points by adding a column of ones
    points_3d_hom = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))

    # Project the 3D points
    points_2d_proj_hom = np.dot(P, points_3d_hom.T).T

    # Convert from homogeneous to 2D coordinates
    points_2d_proj = points_2d_proj_hom[:, :2] / points_2d_proj_hom[:, 2][:, np.newaxis]

    return points_2d_proj

def get_projection_matrix(points2d, points3d, optimize: Optional[bool] = True):
    '''
    Computes projection matrix based on provided control points correspondence
    between image and world spaces.

    Args:
    points2d - control points in image space with shape n x 2, where n - number of points
    points3d - control points in world space with shape n x 3, where n - number of points
    n must be greater or equal to 6
    optimize - wether or not to fine-tune projection matrix with geometric optimization

    return projection matrix of shape 3 x 4
    '''

    assert points2d.shape[0] == points3d.shape[0] and points2d.shape[0] >= 6, print('Shape missmatch')

    T, points_2d_normalized = normalize_points_2d(points2d)
    U, points_3d_normalized = normalize_points_3d(points3d)
    P = dlt_algorithm(points_2d_normalized, points_3d_normalized)

    if optimize:
        P_flattened = P.flatten()

        # Perform the optimization to minimize reprojection error
        res = minimize(
            reprojection_error, P_flattened,
            args=(points_2d_normalized, points_3d_normalized),
        )

        # Reshape the result to a 3x4 projection matrix
        P = res.x.reshape((3, 4))

    # Apply the denormalization
    P = np.dot(np.linalg.inv(T), P).dot(U)

    return P
