#!/usr/bin/env python3
'''Functions for creating dashboard'''
import time
from math import cos, sin
import cv2
import me416_utilities as mu
import numpy as np
import robot_model

# Predefined arrow shape
VERTICES_ARROW = np.array([[-20, 0, 20, 0], [20, -20, 20, 10]])

def vertices_transform(vertices_input, theta, translation):
    """Applies a rigid transformation (rotation + translation) to a set of vertices."""
    rotation_mat = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    vertices_output = rotation_mat @ vertices_input + translation
    return vertices_output


def polygon_draw(img, vertices):
    """Draws a closed polygon on an image."""
    # check for image
    if img is None:
        print('Error: image not loaded')
        return
    # convert vertices to int type
    int_vertices = vertices.astype(int)
    int_vertices = int_vertices.reshape((-1, 1, 2))
    cv2.polylines(img, [int_vertices], isClosed = True,color = (0,0,255), thickness = 5)

def odometry_dashboard():
    """Runs a real-time loop to update and display the odometry dashboard."""
    odometry = Odometry()
    previous_time = time.perf_counter()
    while True:
        current_time = time.perf_counter()
        elapsed_time = current_time - previous_time
        previous_time = current_time

        odometry.step(time_stepsize=elapsed_time)
        dashboard_draw(odometry.theta, odometry.speed_linear, odometry.speed_angular)

        time.sleep(0.2)


def dashboard_draw(theta_heading, speed_linear, speed_angular):
    """Generates and displays a dashboard with transformed arrow and labels."""
    # Load dashboard image
    img_dashboard = cv2.imread('dashboard.png')

    if img_dashboard is None:
        raise FileNotFoundError("dashboard.png not found.")

    # Transform arrow shape
    translation = np.array([[150], [140]])
    transformed_vertices = vertices_transform(VERTICES_ARROW, theta_heading, translation)

    # DEBUG by checking translation with red dot
    #cv2.circle(img_dashboard, (int(translation[0]), int(translation[1])), 5, (0, 0, 255), -1)

    # Draw the arrow on the dashboard
    polygon_draw(img_dashboard, transformed_vertices)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6

    # Add text labels
    cv2.putText(img_dashboard, 'Heading', (208, 45), font, font_scale, (255, 255, 255), 2)
    cv2.putText(img_dashboard, 'Linear speed', (373, 45), font, font_scale, (255, 255, 255), 2)
    cv2.putText(img_dashboard, 'Angular speed', (552, 45), font, font_scale, (255, 255, 255), 2)

    font_scale = 1.5
    # Display numeric values
    cv2.putText(img_dashboard, f'{speed_linear:+.2f}',(357, 140), font, font_scale, (0, 255, 0), 3)
    cv2.putText(img_dashboard, f'{speed_angular:+.2f}',(545, 140), font, font_scale, (0, 255, 0), 3)

    # Show the dashboard
    cv2.imshow('Dashboard', img_dashboard)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


class Odometry:
    """Class to estimate the robot's odometry from wheel encoder readings."""

    def __init__(self):
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.encoder_left = mu.QuadEncoderLeft()
        self.encoder_right = mu.QuadEncoderRight()
        self.theta = 0.0
        self.k_encoder = 1.0e-3

    def step(self, time_stepsize):
        """Updates the estimate of the heading angle using encoders and Euler’s method."""
        speed_left = self.encoder_left.get_speed() * self.k_encoder
        speed_right = self.encoder_right.get_speed() * self.k_encoder
        self.speed_linear, self.speed_angular = robot_model.speeds_to_twist(speed_left, speed_right)
        state_z = np.array([0., 0., self.theta]) # only care about angle
        input_u = [self.speed_linear, self.speed_angular]
        self.theta=self.theta+time_stepsize*robot_model.euler_step(state_z,input_u,time_stepsize)[-1]

if __name__ == "__main__":
    odometry_dashboard()
