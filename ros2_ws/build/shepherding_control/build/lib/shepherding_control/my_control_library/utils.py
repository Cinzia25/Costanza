from cvxopt import matrix
from cvxopt.solvers import qp

import numpy as np
from scipy.special import comb
import torch

def select_targets(herder_pos, target_pos, num_herders):

    xi = 1000

    # Expand dimensions of herder_pos and target_pos to enable broadcasting
    herder_pos_exp = herder_pos[:, np.newaxis, :]  # Shape (N, 1, 2)
    target_pos_exp = target_pos[np.newaxis, :, :]  # Shape (1, M, 2)

    # Calculate the relative positions
    relative_positions = target_pos_exp - herder_pos_exp  # Shape (N, M, 2)

    # Compute the Euclidean distances between herders and targets
    distances = np.linalg.norm(relative_positions, axis=2)  # Shape (N, M)

    # Find the index of the closest herder for each target
    closest_herders = np.argmin(distances, axis=0)  # Shape (M,)

    # Create a boolean mask where each target is only considered if it's closer to the current herder
    closest_mask = np.zeros_like(distances, dtype=bool)
    np.put_along_axis(closest_mask, closest_herders[np.newaxis, :], True, axis=0)

    # Create a boolean mask where distances are less than xi and the herder is the closest one
    mask = (distances < xi) & closest_mask  # Shape (N, M)

    # Calculate the absolute distances from the origin for the targets
    absolute_distances = np.linalg.norm(target_pos, axis=1)  # Shape (M,)

    # Use broadcasting to expand the absolute distances to match the shape of the mask
    expanded_absolute_distances = np.tile(absolute_distances, (num_herders, 1))  # Shape (N, M)

    # Apply the mask to get valid distances only
    valid_absolute_distances = np.where(mask, expanded_absolute_distances, -np.inf)  # Shape (N, M)

    # Find the index of the target with the maximum absolute distance from the origin for each herder
    selected_target_indices = np.argmax(valid_absolute_distances, axis=1)  # Shape (N,)

    # Create a mask to identify herders that have no valid targets
    no_valid_target_mask = np.all(~mask, axis=1)

    # Replace invalid indices with -1 (indicating no target)
    selected_target_indices = np.where(no_valid_target_mask, -1, selected_target_indices)

    return selected_target_indices


def select_targets_learning2(herder_pos, target_pos, num_herders, num_targets, model):

    scaling = 1 / 50 * 20

    herder_pos = herder_pos.copy()
    target_pos = target_pos.copy()

    # Normalize positions using the environment scaling factor
    herder_pos *= scaling
    target_pos *= scaling

    # Compute pairwise distances between all herders (excluding self)
    distances_between_herders = np.linalg.norm(
        herder_pos[:, np.newaxis, :] - herder_pos[np.newaxis, :, :], axis=2
    )
    np.fill_diagonal(distances_between_herders, np.inf)  # Exclude self by setting the diagonal to infinity

    # Identify the indices of the `num_closest_herders` for each herder
    closest_herders_indices = np.sort(np.argsort(distances_between_herders, axis=1)[:, :1],
                                           axis=1)

    # Retrieve and flatten the positions of the closest herders
    closest_herders = herder_pos[closest_herders_indices].reshape(num_herders,
                                                                       -1)  # (num_herders, num_closest_herders * 2)

    # Compute distances between herders and all targets
    distances_to_targets = np.linalg.norm(
        herder_pos[:, np.newaxis, :] - target_pos[np.newaxis, :, :], axis=2
    )

    # Identify the indices of the `num_closest_targets` for each herder
    closest_targets_indices = np.sort(np.argsort(distances_to_targets, axis=1)[:, :5],
                                           axis=1)

    # Retrieve positions of the closest targets
    closest_targets = target_pos[closest_targets_indices]  # (num_herders, num_closest_targets, 2)

    # Select the distances corresponding to the chosen closest target indices
    selected_distances = distances_to_targets[np.arange(num_herders)[:, None], closest_targets_indices]


    # Flatten closest target positions into a single feature vector
    closest_targets_flat = closest_targets.reshape(num_herders, -1)  # (num_herders, num_closest_targets * 2)

    obs = np.concatenate([herder_pos, closest_herders, closest_targets_flat], axis=1)

    obs = torch.tensor(obs, dtype=torch.float32)

    selection = model.get_action(obs)

    return selection





def create_single_integrator_barrier_certificate_with_boundary(barrier_gain=100, safety_radius=0.17, magnitude_limit=0.2, boundary_points = np.array([-1.6, 1.6, -1.0, 1.0])):
    """Creates a barrier certificate for a single-integrator system with a rectangular boundary included.  This function
    returns another function for optimization reasons.

    barrier_gain: double (controls how quickly agents can approach each other.  lower = slower)
    safety_radius: double (how far apart the agents will stay)
    magnitude_limit: how fast the robot can move linearly.

    -> function (the barrier certificate function)
    """

    #Check user input types
    assert isinstance(barrier_gain, (int, float)), "In the function create_single_integrator_barrier_certificate, the barrier gain (barrier_gain) must be an integer or float. Recieved type %r." % type(barrier_gain).__name__
    assert isinstance(safety_radius, (int, float)), "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(safety_radius).__name__
    assert isinstance(magnitude_limit, (int, float)), "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(magnitude_limit).__name__

    #Check user input ranges/sizes
    assert barrier_gain > 0, "In the function create_single_integrator_barrier_certificate, the barrier gain (barrier_gain) must be positive. Recieved %r." % barrier_gain
    assert safety_radius >= 0.12, "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m) plus the distance to the look ahead point used in the diffeomorphism if that is being used. Recieved %r." % safety_radius
    assert magnitude_limit > 0, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= 0.2, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit


    def f(dxi, x):
        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the single-integrator robot velocity command (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(x, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(x).__name__

        #Check user input ranges/sizes
        assert x.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the single integrator robot states (x) must be 2 ([x;y]). Recieved dimension %r." % x.shape[0]
        assert dxi.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the robot single integrator velocity command (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert x.shape[1] == dxi.shape[1], "In the function created by the create_single_integrator_barrier_certificate function, the number of robot states (x) must be equal to the number of robot single integrator velocity commands (dxi). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (x.shape[0], x.shape[1], dxi.shape[0], dxi.shape[1])

        
        # Initialize some variables for computational savings
        N = dxi.shape[1]
        num_constraints = int(comb(N, 2)) + 4*N
        A = np.zeros((num_constraints, 2*N))
        b = np.zeros(num_constraints)
        #H = sparse(matrix(2*np.identity(2*N)))
        H = 2*np.identity(2*N)

        count = 0
        for i in range(N-1):
            for j in range(i+1, N):
                error = x[:, i] - x[:, j]
                h = (error[0]*error[0] + error[1]*error[1]) - np.power(safety_radius, 2)

                A[count, (2*i, (2*i+1))] = -2*error
                A[count, (2*j, (2*j+1))] = 2*error
                b[count] = barrier_gain*np.power(h, 3)

                count += 1
        
        for k in range(N):
            #Pos Y
            A[count, (2*k, 2*k+1)] = np.array([0,1])
            b[count] = 0.4*barrier_gain*(boundary_points[3] - safety_radius/2 - x[1,k])**3;
            count += 1

            #Neg Y
            A[count, (2*k, 2*k+1)] = -np.array([0,1])
            b[count] = 0.4*barrier_gain*(-boundary_points[2] - safety_radius/2 + x[1,k])**3;
            count += 1

            #Pos X
            A[count, (2*k, 2*k+1)] = np.array([1,0])
            b[count] = 0.4*barrier_gain*(boundary_points[1] - safety_radius/2 - x[0,k])**3;
            count += 1

            #Neg X
            A[count, (2*k, 2*k+1)] = -np.array([1,0])
            b[count] = 0.4*barrier_gain*(-boundary_points[0] - safety_radius/2 + x[0,k])**3;
            count += 1
        
        # Threshold control inputs before QP
        norms = np.linalg.norm(dxi, 2, 0)
        idxs_to_normalize = (norms > magnitude_limit)
        dxi[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]

        f = -2*np.reshape(dxi, (2*N,1), order='F')
        b = np.reshape(b, (count,1), order='F')
        result = qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']
        #result = solver2.solve_qp(H, f, A, b, 0)[0]

        return np.reshape(result, (2, N), order='F')

    return f


def create_si_to_uni_dynamics(linear_velocity_gain=1, angular_velocity_limit=np.pi):
    """ Returns a function mapping from single-integrator to unicycle dynamics with angular velocity magnitude restrictions.

        linear_velocity_gain: Gain for unicycle linear velocity
        angular_velocity_limit: Limit for angular velocity (i.e., |w| < angular_velocity_limit)

        -> function
    """

    #Check user input types
    assert isinstance(linear_velocity_gain, (int, float)), "In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be an integer or float. Recieved type %r." % type(linear_velocity_gain).__name__
    assert isinstance(angular_velocity_limit, (int, float)), "In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must be an integer or float. Recieved type %r." % type(angular_velocity_limit).__name__

    #Check user input ranges/sizes
    assert linear_velocity_gain > 0, "In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be positive. Recieved %r." % linear_velocity_gain
    assert angular_velocity_limit >= 0, "In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must not be negative. Recieved %r." % angular_velocity_limit
    

    def si_to_uni_dyn(dxi, poses):
        """A mapping from single-integrator to unicycle dynamics.

        dxi: 2xN numpy array with single-integrator control inputs
        poses: 2xN numpy array with single-integrator poses

        -> 2xN numpy array of unicycle control inputs
        """

        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert poses.shape[0] == 3, "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxi.shape[1] == poses.shape[1], "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size %r x %r and current pose array of size %r x %r." % (dxi.shape[0], dxi.shape[1], poses.shape[0], poses.shape[1])

        M,N = np.shape(dxi)

        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = linear_velocity_gain*(a*dxi[0, :] + b*dxi[1, :])
        dxu[1, :] = angular_velocity_limit*np.arctan2(-b*dxi[0, :] + a*dxi[1, :], dxu[0, :])/(np.pi/2)

        return dxu

    return si_to_uni_dyn



def create_si_to_uni_mapping(projection_distance=0.05, angular_velocity_limit = np.pi):
    """Creates two functions for mapping from single integrator dynamics to 
    unicycle dynamics and unicycle states to single integrator states. 
    
    This mapping is done by placing a virtual control "point" in front of 
    the unicycle.

    projection_distance: How far ahead to place the point
    angular_velocity_limit: The maximum angular velocity that can be provided

    -> (function, function)
    """

    #Check user input types
    assert isinstance(projection_distance, (int, float)), "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    assert isinstance(angular_velocity_limit, (int, float)), "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be an integer or float. Recieved type %r." % type(angular_velocity_limit).__name__
    
    #Check user input ranges/sizes
    assert projection_distance > 0, "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be positive. Recieved %r." % projection_distance
    assert projection_distance >= 0, "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be greater than or equal to zero. Recieved %r." % angular_velocity_limit

    def si_to_uni_dyn(dxi, poses):
        """Takes single-integrator velocities and transforms them to unicycle
        control inputs.

        dxi: 2xN numpy array of single-integrator control inputs
        poses: 3xN numpy array of unicycle poses

        -> 2xN numpy array of unicycle control inputs
        """

        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert poses.shape[0] == 3, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxi.shape[1] == poses.shape[1], "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size %r x %r and current pose array of size %r x %r." % (dxi.shape[0], dxi.shape[1], poses.shape[0], poses.shape[1])


        M,N = np.shape(dxi)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = (cs*dxi[0, :] + ss*dxi[1, :])
        dxu[1, :] = (1/projection_distance)*(-ss*dxi[0, :] + cs*dxi[1, :])

        #Impose angular velocity cap.
        dxu[1,dxu[1,:]>angular_velocity_limit] = angular_velocity_limit
        dxu[1,dxu[1,:]<-angular_velocity_limit] = -angular_velocity_limit 

        return dxu

    def uni_to_si_states(poses):
        """Takes unicycle states and returns single-integrator states

        poses: 3xN numpy array of unicycle states

        -> 2xN numpy array of single-integrator states
        """

        _,N = np.shape(poses)

        si_states = np.zeros((2, N))
        si_states[0, :] = poses[0, :] + projection_distance*np.cos(poses[2, :])
        si_states[1, :] = poses[1, :] + projection_distance*np.sin(poses[2, :])

        return si_states

    return si_to_uni_dyn, uni_to_si_states













