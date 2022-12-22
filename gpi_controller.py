import numpy as np
from main import *

# # Global variable Definitions
n_t     = 200
n_x     = 13
n_y     = 13
n_theta = 13
n_v     = 11
n_w     = 21
gamma   = 0.8
Q       = np.eye(2)
R       = np.eye(2)
q       = 1
epsilon = 0.001

def gpi_controller(_cur_state, _ref_state, _iteration):

    t, x, y, theta      = discretize_state()
    v, w                = discretize_control()
    timestamp           = _iteration + t[0]
    value_function_list = []
    controls_list       = []
    e_t                 = np.asarray(_cur_state - _ref_state).reshape(3,1)
    threshold           = np.inf
    counter             = 0

    # Value Iteration loop
    while threshold > epsilon:

        _ref_state      = np.asarray(_ref_state).reshape(3,1)
        V               = np.zeros((n_v, n_w))

        for i in range(n_v):
            for j in range(n_w):
                u           = np.array([v[i], w[j]]).reshape(2,1)
                e_t_plus_1  = calculate_error_state(e_t, _ref_state, u, timestamp)
                V[i,j]      = gamma * ( e_t_plus_1[0:2, 0].T @ Q @ e_t_plus_1[0:2, 0] + q * ( (1 - np.cos(e_t_plus_1[2]))**2 ) + u.T @ R @ u )

        v_index = np.where(V == np.min(V))[0]
        w_index = np.where(V == np.min(V))[1]
        temp_v  = v[v_index][0]
        temp_w  = w[w_index][0]
        u_min   = np.array([temp_v, temp_w]).reshape(2,1)
        value_function_list.append(np.min(V))
        controls_list.append(u_min)

        e_t         = modify_error(e_t, u_min, _ref_state, timestamp, x, y, theta)
        counter     = counter + 1
        timestamp   = timestamp + t[counter]
        _ref_state  = lissajous(timestamp)

        if len(value_function_list) > 2:
            threshold = abs(value_function_list[-1] - value_function_list[-2])

    v =  controls_list[0][0][0]
    w =  controls_list[0][1][0]

    return [v,w]

def discretize_state():

    _t      = np.linspace(0, 50, n_t)
    _x      = np.linspace(-3, 3, n_x)
    _y      = np.linspace(-3, 3, n_y)
    _theta  = np.linspace(-np.pi, np.pi, n_theta)

    return _t, _x, _y, _theta

def discretize_control():

    _v      = np.linspace(0, 1, n_v)
    _w      = np.linspace(-1, 1, n_w)

    return _v, _w

def calculate_error_state(_cur_error, _ref_state, _u, _timestamp):

    alpha           = _ref_state[2]
    theta_tilda     = _cur_error[2]
    G               = np.zeros((3,2))
    G[0,0]          = time_step * np.cos(theta_tilda + alpha)
    G[1,0]          = time_step * np.sin(theta_tilda + alpha)
    G[2,1]          = time_step
    next_ref_state  = np.asarray(lissajous(_timestamp + time_step)).reshape(3,1)

    _error_state    = _cur_error + G @ _u + (_ref_state - next_ref_state)

    return _error_state

def modify_error(_e_t, _u, _ref_state, _timestamp, x, y, theta):

    e_t_plus_1      = calculate_error_state(_e_t, _ref_state, _u, _timestamp)
    _ref_state      = lissajous(_timestamp + time_step) # Ref state at next timestamp
    _ref_state      = np.asarray(_ref_state).reshape(3,1)
    state_t_plus_1  = e_t_plus_1 + _ref_state
    _x              = x[ min( range(len(x)), key = lambda i: abs(x[i]-state_t_plus_1[0,0]) ) ]
    _y              = y[ min( range(len(y)), key = lambda i: abs(y[i]-state_t_plus_1[1,0]) ) ]
    _theta          = theta[ min( range(len(theta)), key = lambda i: abs(theta[i]-state_t_plus_1[2,0]) ) ]
    _state          = np.array([_x, _y, _theta]).reshape(3,1)
    _e_t            = _state - _ref_state

    return _e_t
