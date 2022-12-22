import numpy as np
from casadi import *
from main import *

def cec_controller(_cur_state, _cur_ref_state, _iteration):

    # Optimizer
    opti        = casadi.Opti()

    # Constants
    T           = 10 #int(5/time_step)

    # Initialize variables
    e_t         = opti.variable(3,T+1)
    e_t[:,0]    = _cur_state - _cur_ref_state
    u           = opti.variable(2,T)

    # Initialize Constant parameters
    Q           = opti.parameter(2,2)   # 2x2 symmetric positive-definite matrix - Stage cost for deviating from reference position trajectory r_t
    R           = opti.parameter(2,2)   # 2x2 symmetric positive-definite matrix - Stage cost for using excessive control effort
    _q          = opti.parameter()      # Scalar - Stage cost for deviating from reference orientation trajectory alpha_t
    gamma       = opti.parameter()      # Discount factor

    # Set values for the Constant parameters
    opti.set_value(Q, 15*np.eye(2))
    opti.set_value(R, 5*np.eye(2))
    opti.set_value(_q, 40)
    opti.set_value(gamma, 0.9)

    # Initialize reference state parameters
    r           = opti.parameter(2, T+1)
    alpha       = opti.parameter(1, T+1)

    # Set values for reference state parameters
    for i in range(T+1):
        #ref_state   = lissajous(i*time_step + _iteration)
        ref_state   = lissajous(i + _iteration)
        opti.set_value(r[:,i], ref_state[0:2])
        opti.set_value(alpha[:,i], ref_state[2])

    # Set bounds on control input u
    opti.subject_to( u[0] >= 0 )         # Lower bound on linear velocity
    opti.subject_to( u[0] <= 1 )         # Upper bound on linear velocity
    opti.subject_to( u[1] >= -1 )        # Lower bound on angular velocity
    opti.subject_to( u[1] <= 1 )         # Upper bound on angular velocity

    sum = 0
    for i in range(T):

        x       = e_t[0,i] + r[0,i]
        y       = e_t[1,i] + r[1,i]
        theta   = e_t[2,i] + alpha[:,i]
        opti.subject_to( (x-1)**2 + (y-2)**2 > (0.5)**2 )   # Constraint to avoid Circle 1
        opti.subject_to( (x+2)**2 + (y+2)**2 > (0.5)**2 )   # Constraint to avoid Circle 2
        opti.subject_to( x >= -3 )                      # Lower bound on x-coordinate
        opti.subject_to( x <= 3 )                       # Upper bound on x-coordinate
        opti.subject_to( y >= -3 )                      # Lower bound on y-coordinate
        opti.subject_to( y <= 3 )                       # Upper bound on y-coordinate
        opti.subject_to( e_t[2,i] >= -pi )              # Lower bound on theta
        opti.subject_to( e_t[2,i] < pi )                # Upper bound on theta


        # Set constraint on e_t+1

        G00 = time_step * cos(theta)
        G01 = 0
        G10 = time_step * sin(theta)
        G11 = 0
        G20 = 0
        G21 = time_step

        G_tilda = horzcat( vertcat(G00, G10, G20), vertcat(G01, G11, G21) )

        opti.subject_to( e_t[:, i+1] == e_t[:, i] + G_tilda @ u[:, i] +  vertcat( (r[:,i] - r[:,i+1]), (alpha[:,i] - alpha[:,i+1]) ) )

        sum = sum + ( gamma**i * ( e_t[0:2,i].T @ Q @ e_t[0:2,i] + _q * ( 1 - cos(e_t[2,i]) )**2 + u[:,i].T @ R @ u[:,i] ) )

    v_star = ( e_t[0:2,T].T @ Q @ e_t[0:2,T] + _q*( 1 - cos(e_t[2,T]) )**2 ) + sum

    # NLP Solver

    opti.minimize(v_star)
    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=0)
    opti.solver("ipopt", p_opts, s_opts)
    sol = opti.solve()

    return [sol.value(u)[0,0],sol.value(u)[1,0]]
