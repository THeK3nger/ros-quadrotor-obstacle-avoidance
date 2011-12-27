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
#include <armadillo>
#include "quadrotor_obstacle_avoidance/functionlib.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358

double mass=0.64;
double Ix=0.02;
double Iy=0.02;
double Iz=0.04;
double Ir=10^-3;
double g=9.81;
double omega=0;
double l=0.165;
double d=4.5* pow(10,-7);



using namespace arma;
using namespace std;

double * controller (double* X, double * V)
{
	double * U = new double[4];
	
	double roll=X[3];
	double pitch=X[4];
	double yaw=X[5];
	
	double p=X[9];
	double q=X[10];    
	double r=X[11];
	double u1=X[12];
	double eta=X[13];
	
	// Computing the inverse of the decoupling matrix J. Its expression
	// has been computed with the symoblic toolbox provided by Matlab 
	// and then copy-pasted into the code.
	mat J_inv=mat(4,4);
	
	// First row.
	J_inv(0,0)= mass*sin(roll)*sin(yaw) + mass*cos(roll)*cos(yaw)*sin(pitch);
	J_inv(0,1)= mass*cos(roll)*sin(pitch)*sin(yaw) - mass*cos(yaw)*sin(roll);
	J_inv(0,2)= mass*cos(pitch)*cos(roll);
	J_inv(0,3)= 0;
	
	// Second row.
	J_inv(1,0)= (Ix*mass*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/(l*u1);
	J_inv(1,1)= -(Ix*mass*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/(l*u1);
	J_inv(1,2)= -(Ix*mass*cos(pitch)*sin(roll))/(l*u1);
	J_inv(1,3)= 0;
	
	// Third row.
	J_inv(2,0)= (Iy*mass*cos(pitch)*cos(yaw))/(l*u1);
	J_inv(2,1)= (Iy*mass*cos(pitch)*sin(yaw))/(l*u1);
	J_inv(2,2)=  -(Iy*mass*sin(pitch))/(l*u1);
	J_inv(2,3)= 0;
	
	// Fourth row.
	J_inv(3,0)= -(Iz*mass*cos(pitch)*cos(yaw)*sin(roll))/(d*u1*cos(roll));
	J_inv(3,1)= -(Iz*mass*cos(pitch)*sin(roll)*sin(yaw))/(d*u1*cos(roll));
	J_inv(3,2)= (Iz*mass*sin(pitch)*sin(roll))/(d*u1*cos(roll));
	J_inv(3,3)= (Iz*cos(pitch))/(d*cos(roll));
	
	
	mat L=mat(4,1);
	
	// Also for the vector L the expression has been computed with the 
	// symoblic toolbox provided by Matlab and then copy-pasted into the code.
	
	L(0,0)= (q*cos(roll) - r*sin(roll))*((u1*(r*cos(roll)*(pow(tan(pitch),2) + 1) \ 
	+ q*sin(roll)*(pow(tan(pitch),2) + 1))*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/mass \
	+ (eta*cos(pitch)*cos(roll)*cos(yaw))/mass - (u1*cos(roll)*sin(yaw)*(r*cos(roll) \
	+ q*sin(roll)))/mass - (u1*cos(pitch)*cos(yaw)*sin(roll)*(p + r*cos(roll)*tan(pitch) \
	+ q*tan(pitch)*sin(roll)))/mass - (u1*cos(roll)*cos(yaw)*sin(pitch)*(q*cos(roll) - r*sin(roll)))/mass \
	+ (u1*sin(pitch)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(r*cos(roll) + q*sin(roll)))/(mass*pow(cos(pitch),2))) \
	- ((Ir*omega*p)/Iy + (p*r*(Ix - Iz))/Iy)*((u1*sin(roll)*(cos(yaw)*sin(roll) \
	- cos(roll)*sin(pitch)*sin(yaw)))/(mass*cos(pitch)) + (u1*cos(pitch)*pow(cos(roll),2)*cos(yaw))/mass \
	+ (u1*tan(pitch)*sin(roll)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/mass) + (p + r*cos(roll)*tan(pitch) \
	+ q*tan(pitch)*sin(roll))*((eta*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/mass - (u1*(sin(roll)*sin(yaw) \
	+ cos(roll)*cos(yaw)*sin(pitch))*(p + r*cos(roll)*tan(pitch) + q*tan(pitch)*sin(roll)))/mass \
	+ (u1*(q*cos(roll)*tan(pitch) - r*tan(pitch)*sin(roll))*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/mass \
	+ (u1*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(r*cos(roll) + q*sin(roll)))/(mass*cos(pitch)) \
	+ (u1*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(q*cos(roll) - r*sin(roll)))/(mass*cos(pitch)) \
	- (u1*cos(pitch)*cos(roll)*cos(yaw)*(r*cos(roll) + q*sin(roll)))/mass - (u1*cos(pitch)*cos(yaw)*sin(roll)*(q*cos(roll) \
	- r*sin(roll)))/mass) + eta*(((cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(p + r*cos(roll)*tan(pitch) \
	+ q*tan(pitch)*sin(roll)))/mass + ((cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(r*cos(roll) \
	+ q*sin(roll)))/(mass*cos(pitch)) + (cos(pitch)*cos(roll)*cos(yaw)*(q*cos(roll) - r*sin(roll)))/mass) \
	+ ((r*cos(roll) + q*sin(roll))*((eta*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/mass + (u1*(cos(roll)*cos(yaw) \
	+ sin(pitch)*sin(roll)*sin(yaw))*(p + r*cos(roll)*tan(pitch) + q*tan(pitch)*sin(roll)))/mass - (u1*(sin(roll)*sin(yaw) \
	+ cos(roll)*cos(yaw)*sin(pitch))*(r*cos(roll) + q*sin(roll)))/(mass*cos(pitch)) \
	- (u1*cos(pitch)*cos(roll)*sin(yaw)*(q*cos(roll) - r*sin(roll)))/mass))/cos(pitch) + (u1*(cos(roll)*sin(yaw) \
	- cos(yaw)*sin(pitch)*sin(roll))*((Ir*omega*q)/Ix + (q*r*(Iy - Iz))/Ix))/mass \
	+ (p*q*(Ix - Iy)*((u1*cos(roll)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/(mass*cos(pitch)) \
	+ (u1*cos(roll)*tan(pitch)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/mass \
	- (u1*cos(pitch)*cos(roll)*cos(yaw)*sin(roll))/mass))/Iz;
	
	L(1,0)=eta*(((sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(r*cos(roll) + q*sin(roll)))/(mass*cos(pitch)) \
	- ((cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(p + r*cos(roll)*tan(pitch) + q*tan(pitch)*sin(roll)))/mass \
	+ (cos(pitch)*cos(roll)*sin(yaw)*(q*cos(roll) - r*sin(roll)))/mass) - ((Ir*omega*p)/Iy \
	+ (p*r*(Ix - Iz))/Iy)*((u1*sin(roll)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/(mass*cos(pitch)) \
	+ (u1*cos(pitch)*pow(cos(roll),2)*sin(yaw))/mass - (u1*tan(pitch)*sin(roll)*(cos(roll)*cos(yaw) \
	+ sin(pitch)*sin(roll)*sin(yaw)))/mass) - (p + r*cos(roll)*tan(pitch) + q*tan(pitch)*sin(roll))*((eta*(cos(roll)*cos(yaw) \
	+ sin(pitch)*sin(roll)*sin(yaw)))/mass - (u1*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(p \
	+ r*cos(roll)*tan(pitch) + q*tan(pitch)*sin(roll)))/mass + (u1*(q*cos(roll)*tan(pitch) \
	- r*tan(pitch)*sin(roll))*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/mass - (u1*(sin(roll)*sin(yaw) \
	+ cos(roll)*cos(yaw)*sin(pitch))*(q*cos(roll) - r*sin(roll)))/(mass*cos(pitch)) - (u1*(cos(roll)*sin(yaw) \
	- cos(yaw)*sin(pitch)*sin(roll))*(r*cos(roll) + q*sin(roll)))/(mass*cos(pitch)) \
	+ (u1*cos(pitch)*sin(roll)*sin(yaw)*(q*cos(roll) - r*sin(roll)))/mass + (u1*cos(pitch)*cos(roll)*sin(yaw)*(r*cos(roll) \
	+ q*sin(roll)))/mass) - (q*cos(roll) - r*sin(roll))*((u1*(r*cos(roll)*(pow(tan(pitch),2) + 1) \
	+ q*sin(roll)*(pow(tan(pitch),2) + 1))*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/mass \
	- (u1*cos(roll)*cos(yaw)*(r*cos(roll) + q*sin(roll)))/mass - (eta*cos(pitch)*cos(roll)*sin(yaw))/mass \
	+ (u1*cos(roll)*sin(pitch)*sin(yaw)*(q*cos(roll) - r*sin(roll)))/mass \
	+ (u1*cos(pitch)*sin(roll)*sin(yaw)*(p + r*cos(roll)*tan(pitch) + q*tan(pitch)*sin(roll)))/mass \
	- (u1*sin(pitch)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(r*cos(roll) + q*sin(roll)))/(mass*pow(cos(pitch),2))) \
	+ ((r*cos(roll) + q*sin(roll))*((eta*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/mass \
	+ (u1*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(p + r*cos(roll)*tan(pitch) \
	+ q*tan(pitch)*sin(roll)))/mass + (u1*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(r*cos(roll) \
	+ q*sin(roll)))/(mass*cos(pitch)) + (u1*cos(pitch)*cos(roll)*cos(yaw)*(q*cos(roll) - r*sin(roll)))/mass))/cos(pitch) \
	- (u1*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*((Ir*omega*q)/Ix + (q*r*(Iy - Iz))/Ix))/mass \
	- (p*q*(Ix - Iy)*((u1*cos(roll)*tan(pitch)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/mass \
	- (u1*cos(roll)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/(mass*cos(pitch)) \
	+ (u1*cos(pitch)*cos(roll)*sin(roll)*sin(yaw))/mass))/Iz;
	
	L(2,0)= ((Ir*omega*p)/Iy + (p*r*(Ix - Iz))/Iy)*((u1*pow(cos(roll),2)*sin(pitch))/mass + (u1*sin(pitch)*pow(sin(roll),2))/mass) \
	- ((eta*cos(pitch)*sin(roll))/mass + (p*u1*cos(pitch)*cos(roll))/mass)*(p + r*cos(roll)*tan(pitch) \
	+ q*tan(pitch)*sin(roll)) - (q*cos(roll) - r*sin(roll))*((eta*cos(roll)*sin(pitch))/mass \
	+ (q*u1*cos(pitch)*pow(cos(roll),2))/mass + (q*u1*cos(pitch)*pow(sin(roll),2))/mass - (p*u1*sin(pitch)*sin(roll))/mass) \
	- eta*((p*cos(pitch)*sin(roll))/mass + (q*pow(cos(roll),2)*sin(pitch))/mass + (q*sin(pitch)*pow(sin(roll),2))/mass) \
	- (u1*cos(pitch)*sin(roll)*((Ir*omega*q)/Ix + (q*r*(Iy - Iz))/Ix))/mass;
	
	L(3,0)= ((q*cos(roll) - r*sin(roll))*(p + r*cos(roll)*tan(pitch) + q*tan(pitch)*sin(roll)))/cos(pitch) \
	- (sin(roll)*((Ir*omega*p)/Iy + (p*r*(Ix - Iz))/Iy))/cos(pitch) + (tan(pitch)*(r*cos(roll) + q*sin(roll))*(q*cos(roll) \
	- r*sin(roll)))/cos(pitch) + (p*q*cos(roll)*(Ix - Iy))/(Iz*cos(pitch));

	mat V_ar = mat(4,1);
	
	V_ar(0,0) = V[0];
	V_ar(1,0) = V[1];
	V_ar(2,0) = V[2];
	V_ar(3,0) = V[3];
	
	// The product between J_inv and (V-L) is computed using the ARMADILLO
	// library.
	mat U_tmp = J_inv * (V_ar-L);
	
	U[0]= U_tmp(0,0);
	U[1]= U_tmp(1,0);
	U[2]= U_tmp(2,0);
	U[3]= U_tmp(3,0);
	
	return U;
}

// Given a vector at time t-1 (X) and at time t (X_new) we compute the derivative.
double* backward_derivative(double* X,double* X_new, double dt){

	
	double* X_p=new double[14];
	
	for(int i=0; i<14; i++){
		X_p[i]=(X_new[i]-X[i])/dt;
	}
	
	return X_p;
	
}

double* force_vector(double* position, double yaw, double* goal, double* obstacle){
	
	double* grad_att=new double[4];
	double* grad_rep=new double[4];
	double* grad=new double[4];
	double ka=5;
	double kr=8;
	double kyaw=5;
	double dmax=6;
	double r = 0.5;
	double gain;
	double ob_distance;	
	
	// Computing the actractive gradient for the position as the derivative of the
	// potential field. The potential U is defined as 1/2*Ka*(distance^2).
	grad_att[0]=-ka*(position[0]-goal[0]);
	grad_att[1]=-ka*(position[1]-goal[1]);
	grad_att[2]=-ka*(position[2]-goal[2]);
    grad_att[3]=-kyaw*(yaw - goal[3]);
	
	// Computing the repulsive gradient.
	ob_distance=sqrt(pow(obstacle[0],2) + pow(obstacle[1],2) + pow(obstacle[2],2))-r;
	
	if(ob_distance<dmax){
		gain = kr*(1/ob_distance - 1/dmax)*(1/pow(ob_distance,3));
	}else{
		gain=0;
	}
	
	grad_rep[0]= obstacle[0]*gain;
	grad_rep[1]= obstacle[1]*gain;
	grad_rep[2]= obstacle[2]*gain;
	grad_rep[3]= 0;
		
	// Computing the total gradient.
	grad[0]=grad_att[0]-grad_rep[0];
	grad[1]=grad_att[1]-grad_rep[1];
	grad[2]=grad_att[2]-grad_rep[2];
	grad[3]=grad_att[3]-grad_rep[3];

	return grad;
}

// As we are using derivative of the second order and higher with potential field
// we need to compute a damping term.
double* damping(double* velocity, double* acceleration, double* jerk, double yaw_d){
	double kvel=40;
	double kacc=20;
	double kjerk=10;
	double kyaw=20;
	
	double* damping_vector=new double[4];
	
	for(int i=0; i<3; i++){
		damping_vector[i]=kvel*velocity[i]+kacc*acceleration[i]+kjerk*jerk[i];
	}
	damping_vector[3]=kyaw*yaw_d;
	
	return damping_vector;
	
}

// Angles in GAZEBO are defined between -PI and PI, in order to avoid 
// discontinuities we "denormalize" them.
double denormalize_angle(double angle, double previous_angle){
	
	double diff;
	double delta;
	
	diff=previous_angle-angle;
	
	// We add or substract 2PI to the angle returned by GAZEBO untill
	// the difference between the angle computed and the previous is minimum.
	if(diff>0){
		do{
			delta=previous_angle-(angle+2*PI);
				if(abs(delta)<abs(diff)){
					diff=delta;
					angle+=2*PI;
				}else{
					return angle;
				}
		}while(true);
	}
	if(diff<0){
		do{
			delta=previous_angle-(angle-2*PI);
				if(abs(delta)<abs(diff)){
					diff=delta;
					angle-=2*PI;
				}else{
					return angle;
				}
		}while(true);
	}
					
	return angle;
	
}

// Function used to print on a file the desired values plotted later on
// with Matlab.
void toMatlab(FILE* id, double* X, double* U, double* FV, double* V){
	
	for(int i=0; i<13; i++){
		fprintf(id,"%f ", X[i]);
	}
	for(int i=0; i<4; i++){
		fprintf(id,"%f ", U[i]);
	}
	for(int i=0; i<4; i++){
		fprintf(id,"%f ", FV[i]);
	}
	for(int i=0; i<4; i++){
		fprintf(id,"%f ", V[i]);
	}
	
	fprintf(id,"\n");
}
