import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib import lines

def get_trajectory_input(initial_state, target_position):
    move_backward = False
    wheel_radius = 20.0 # 20mm 
    wheel_distance = 85.0 # 85mm
    init_x = float(initial_state[0])
    init_y = float(initial_state[1])
    init_theta = (initial_state[2]+2*np.pi)%(2*np.pi)

    target_x = float(target_position[0])
    target_y = float(target_position[1])

    d = np.sqrt((target_x-init_x)**2 + (target_y-init_y)**2) # distance between initial position to target position
    d_angle = np.arctan2((target_y-init_y),(target_x-init_x)) # angle of d vector w.r.t global frame

    d_angle = (d_angle+2*np.pi)%(2*np.pi) # keep it between 0 to 360

    # print init_theta, d_angle

    if init_theta == d_angle or init_theta == (d_angle+np.pi)%(2*np.pi): # if d vector and tangent line overlap
        omega_l = d/wheel_radius
        omega_r = omega_l

        trajectory_obj = lines.Line2D([init_x,target_x], [init_y,target_y], linewidth=1, color='b') # line object to plot straight trajectory

        if init_theta == (d_angle+np.pi)%(2*np.pi): # d vector and tangent line are 180 degrees apart
            move_backward = True

    else: # if d vector and tangent line does not align, robot follows a curved trajectory.
        delta_angle = ((d_angle - init_theta)+2*np.pi)%(2*np.pi)
        if delta_angle > np.pi:
            delta_angle = delta_angle - 2*np.pi

        if abs(delta_angle) > np.pi/2.0: # if robot needs to turn more than 90 degrees, let it move backwards
            move_backward = True
            init_theta = init_theta+np.pi # flip the robot direction by 180 degrees

            ## update delta_angle
            delta_angle = ((d_angle - init_theta)+2*np.pi)%(2*np.pi)
            if delta_angle > np.pi:
                delta_angle = delta_angle - 2*np.pi

        angle_btw_d_tan = abs(d_angle - init_theta) # angle between d vector and tangent line (in the direction robot is facing)
        angle = abs(np.pi - 2.0*(np.pi/2.0-angle_btw_d_tan)) # angle of the arc
        radius = abs(d/(2.0*np.sin(angle/2.0))) # radius of the arc

        if delta_angle > 0: # robot is moving counter-clockwise
            # Given that the robot travels to the target in 1 second
            omega_l = (radius - wheel_distance/2.0)*angle/wheel_radius # angular velocity of left wheel
            omega_r = (radius + wheel_distance/2.0)*angle/wheel_radius # angular velocity of right wheel
            angle_radius = init_theta + np.pi/2.0 # orientation of the line connecting circle center and initial point
            theta1 = angle_radius - np.pi
            theta2 = theta1 + angle
        else: # robot is moving clockwise
            # Given that the robot travels to the target in 1 second
            omega_l = (radius + wheel_distance/2.0)*angle/wheel_radius # angular velocity of left wheel
            omega_r = (radius - wheel_distance/2.0)*angle/wheel_radius # angular velocity of right wheel
            angle_radius = init_theta - np.pi/2.0 # orientation of the line connecting circle center and initial point
            theta2 = angle_radius + np.pi
            theta1 = theta2 - angle
            # print omega_l, omega_r

        # slope = np.tan(angle_radius);
        delta_x = radius * np.cos(angle_radius)
        delta_y = radius * np.sin(angle_radius)

        center_x = init_x + delta_x
        center_y = init_y + delta_y

        theta1 = (theta1+2*np.pi)%(2*np.pi) # keep it between 0 to 360
        theta2 = (theta2+2*np.pi)%(2*np.pi) # keep it between 0 to 360
        if theta2-theta1 < 180:
            trajectory_obj = patches.Arc((center_x, center_y), radius*2, radius*2,
                         theta1=theta1*180.0/np.pi, theta2=theta2*180.0/np.pi, linewidth=1, color='b') # arc object to plot curved trajectory
        else:
            trajectory_obj = patches.Arc((center_x, center_y), radius*2, radius*2,
                         theta1=theta2*180.0/np.pi, theta2=theta1*180.0/np.pi, linewidth=1, color='b') # arc object to plot curved trajectory
        # print round(d), 'init_theta: ', round(init_theta*180/np.pi), 'd_angle: ', round(d_angle*180/np.pi), 'delta angle', round(delta_angle*180/np.pi)#round(angle_btw_d_tan*180/np.pi), round(angle*180/np.pi) # for debugging
        # print round(angle_radius*180.0/np.pi), 'center: ', round(center_x), round(center_y), 'thetas: ', round(theta1*180.0/np.pi), round(theta2*180.0/np.pi), round(radius)

    if move_backward:
        orig_omega_l = omega_l
        omega_l = -omega_r
        omega_r = -orig_omega_l

    final_angle = d_angle # orientation of robot at the goal

    return trajectory_obj, omega_l, omega_r, final_angle, d