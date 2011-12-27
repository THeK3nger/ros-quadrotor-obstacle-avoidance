/**
    This file is part of ROS Quadrotor Feedback Controller.

    Foobar is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    ROS Quadrotor Feedback Controller is distributed in the hope that it will
    be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
    Public License for more details.

    You should have received a copy of the GNU General Public License along
    with ROS Quadrotor Feedback Controller; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
**/
#ifndef FUNCTIONLIB_HH
#define FUNCTIONLIB_HH

double * controller (double* X, double *V, FILE* id );

double* backward_derivative(double* X,double* X_new, double dt);

double* force_vector(double* position, double yaw, double* goal, double* obstacle);

double* damping(double* velocity, double* acceleration, double* jerk, double yaw_d);

double denormalize_angle(double angle, double previous_angle);

void toMatlab(FILE* id, double* X, double* U, double* FV, double* V);
#endif
