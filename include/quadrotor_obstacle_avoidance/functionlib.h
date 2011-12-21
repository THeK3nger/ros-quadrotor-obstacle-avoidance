#ifndef FUNCTIONLIB_HH
#define FUNCTIONLIB_HH

double * controller (double* X, double *V, FILE* id );

double* backward_derivative(double* X,double* X_new, double dt);

double* force_vector(double* position, double yaw, double* goal, double* look_at, double* obstacle);

double* damping(double* velocity, double* acceleration, double* jerk, double yaw_d);

double denormalize_angle(double angle, double previous_angle);

void toMatlab(FILE* id, double* X, double* U, double* FV, double* V);
#endif
