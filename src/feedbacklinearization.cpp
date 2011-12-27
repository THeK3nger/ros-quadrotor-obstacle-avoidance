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

#include <algorithm>
#include <assert.h>

#include <quadrotor_obstacle_avoidance/feedbacklinearization.h>
#include <quadrotor_obstacle_avoidance/functionlib.h>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/Quatern.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/PhysicsEngine.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

/*
  Utility Macros
*/

#define STAMPA(X,Y) printf("%s : %f\n",(X),(Y))
#define LINE printf("----------\n")

#define MAX 20      // MAX THRUST
#define MIN 1       // MIN THRUST
#define MAX_ROT 15  // MAX WRENCH
#define MIN_ROT -15 // MIN WRENCH

using namespace gazebo;

/**
  This macro link this controller to Gazebo core.
**/
GZ_REGISTER_DYNAMIC_CONTROLLER("feedback_linearization", FeedbackLinearization);

////////////////////////////////////////////////////////////////////////////////
// Constructor
FeedbackLinearization::FeedbackLinearization(Entity *parent) : Controller(parent)
{

    this->parent_ = dynamic_cast<Model*> (parent);

    if (!parent_) {
        gzthrow("Controller_Test controller requires a Model as its parent");
    }

    this->X=new double[14];					// state returned by gazebo at time t.
    this->X_prev=new double[14];			// state returned by gazebo at time t-1.
    this->X_d=new double[14];				// first order derivative of the state at time t.
    this->X_d_prev=new double[14];			// first order derivative of the expanded state at time t-1.
    this->X_dd_prev=new double[14];			// second order derivative of the expanded state at time t-1.
    this->U_tilde= new double[4];			// input of the expanded model.
    this->Eta = 0;							// ETA.
    this->U1_reale = 6.5;					// U1 of the real quadrotor.

    this->goal=new double[4];               // Goal Coordinate (goal in configuration space)
    this->obstacle=new double[3];           // Near Obstacle Coordinate

    this->id=fopen("/home/vittorio/Scrivania/presentazione/ROS/quadrotor_obstacle_avoidance/toMatlab.txt", "w");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
FeedbackLinearization::~FeedbackLinearization()
{
    delete goal_callback_queue_thread_;
    delete obstacle_callback_queue_thread_;
    delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void FeedbackLinearization::LoadChild(XMLConfigNode *node)
{
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "quadrotor_feedback_linearization", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    rosnode_ = new ros::NodeHandle("/");


    // Setting options of the goal's subscriber.
    ros::SubscribeOptions sg_options = ros::SubscribeOptions::create<geometry_msgs::Point>("goal", 1,
                                                          boost::bind(&FeedbackLinearization::goalCallback, this, _1),
                                                          ros::VoidPtr(), &queue_goal);

    // Subcribing the topic in which the goal will be published.
    sub_goal = rosnode_->subscribe(sg_options);

    // Setting options for the laser subscriber.
    ros::SubscribeOptions so_options = ros::SubscribeOptions::create<sensor_msgs::PointCloud>("/full_cloud", 1,
                                                          boost::bind(&FeedbackLinearization::obstacleCallback, this, _1),
                                                          ros::VoidPtr(), &queue_obstacle);

    // Subcribing the topic in which the laser messages will be published.
    sub_obstacle = rosnode_->subscribe(so_options);

    body_main = this->parent_->GetBody("chassis_body");


    // WARNING: both X and X_tilde shuld be initialized in such a way to be coherent with the launch file.
    for(int i=0; i<14; i++){
        X_prev[i]=0;
    }
    X_prev[0]=0;
    X_prev[2]=7;

    for(int i=0; i<14; i++){
        X[i]=0;
        X_d[i]=0;
        X_d_prev[i]=0;
        X_dd_prev[i]=0;
    }
    X[0]=0;
    X[2]=7;
    X[12]=6.0;
    X_prev[0]=0;
    X_prev[2]=7;
    X_prev[12]=6.0;

    for(int i=0; i<4; i++){
        U_tilde[i]=0;
    }

    // The goal point is initialized to simulate a take off
    // phase.
    this->goal[0]=20;
    this->goal[1]=20;
    this->goal[2]=7;
    this->goal[3]=0;

    // The obstacle is initialized at [1000, 1000, 1000] in order to
    // avoid any problem during the "take off" phase.
    this->obstacle[0]=1000;
    this->obstacle[1]=1000;
    this->obstacle[2]=1000;

}

void FeedbackLinearization::InitChild()
{
      goal_callback_queue_thread_ = new boost::thread(boost::bind(&FeedbackLinearization::GoalQueueThread, this));
      obstacle_callback_queue_thread_ = new boost::thread(boost::bind(&FeedbackLinearization::ObstacleQueueThread, this));
}

void FeedbackLinearization::SaveChild(std::string &prefix, std::ostream &stream)
{

}

void FeedbackLinearization::ResetChild()
{

}

void FeedbackLinearization::UpdateChild()
{
    //Reading actual state from Gazebo.
    Pose3d pose=this->body_main->GetWorldPose(); 					// World Pose.
    Vector3 linear_velocity=this->body_main->GetWorldLinearVel();   // Linear Velocity.
    Vector3 angular_velocity=this->body_main->GetWorldAngularVel(); // Angular Velocity.
    Quatern frame_rot=pose.rot;
    Vector3 rpy=frame_rot.GetAsEuler(); 							// Converting pose from quaternion to rpy angles.
    double dt=0.001;  												// Time step used to compute numeric derivative or integral.

    // Saving in X_prev the previous state of the quadrotor.
    for(int i=0; i<14; i++){
        X_prev[i]=X[i];
    }
    // Loading the actual state from GAZEBO.
    X[0]= pose.pos.x;
    X[1]= pose.pos.y;
    X[2]= pose.pos.z;
    X[3]= rpy.x;
    X[4]= rpy.y;
    X[5]= rpy.z;
    X[6]= linear_velocity.x;
    X[7]= linear_velocity.y;
    X[8]= linear_velocity.z;
    X[9]= angular_velocity.x;
    X[10]= angular_velocity.y;
    X[11]= angular_velocity.z;
    X[12]= this->U1_reale;
    X[13]= this->Eta;

    // Angles in GAZEBO are normalized between -PI and PI in order to
    // get the real values we denormalize them.
    X[5]=denormalize_angle(X[5],X_prev[5]);

    // Saving the previous derivative of the state.
    for(int i=0; i<14; i++){
        X_d_prev[i]=X_d[i];
    }

    // Deleting the previous derivative of the state in order to compute
    // the new one.
    delete X_d;
    X_d=backward_derivative(X_prev,X,dt);

    // Compute the second dertivative of the state.
    double* X_dd=backward_derivative(X_d_prev, X_d, dt);
    for(int i=0; i<14; i++){
        X_dd_prev[i]=X_dd[i];
    }

    // Computing the feedforward term using the first and second order
    // derivative.
    double position[3]={X[0], X[1], X[2]};
    double velocity[3]={X[6], X[7], X[8]};

    double acc[3]={X_dd[0], X_dd[1], X_dd[2]};
    double jerk[3]={X_dd[6], X_dd[7], X_dd[8]};

    double* FV=force_vector(position, X[5], this->goal, this->obstacle);
    double* DP=damping(velocity, acc, jerk, X[11]);

    // Computing the V vector from the PDDDD.
    double* V=new double[4];
    for (int i=0; i<4; i++){
        V[i]=FV[i]-DP[i];
    }

    // Deleting the previous U_tilde vector in order to compute the new one.
    delete U_tilde;
    U_tilde=controller(X, V, id);

    // Numerical integration of ETA and U1.
    this->Eta = this->Eta+(U_tilde[0])*dt;
    this->U1_reale = this->U1_reale+(this->Eta)*dt;

    // Saving some usefull variables.
    toMatlab(id, X, U_tilde, FV, V);

    delete FV;
    delete DP;

    // Saturating input.
    FeedbackLinearization::saturationForce();

    // Rotating the forces so the results are expressed in the quadrotor
    // reference frame and then applying them.
    Vector3* v = new Vector3(0.0,0.0,(this->U1_reale));
    Vector3 torque(U_tilde[1], U_tilde[2], U_tilde[3]);

    Vector3 actual_force=frame_rot.RotateVector(*v);


    Vector3 actual_torque=frame_rot.RotateVector(torque);

    this->body_main->SetForce(actual_force);
    this->body_main->SetTorque(torque);

}


void FeedbackLinearization::FiniChild()
{

}

////////////////////////////////////////////////////////////////////////////////
// Callback function for the goal topic
void FeedbackLinearization::goalCallback(const geometry_msgs::Point::ConstPtr& goal_msg)
{

  lock.lock();

  this->goal[0] = goal_msg->x;
  this->goal[1] = goal_msg->y;
  this->goal[2] = goal_msg->z;
  this->goal[3] = 0; // TODO: Change message from Point to Pose and set this with `yaw`.

  lock.unlock();

}

void FeedbackLinearization::GoalQueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_goal.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Callback function for the obstacle topic
void FeedbackLinearization::obstacleCallback(const sensor_msgs::PointCloud::ConstPtr& obstacle_msg)
{

  lock.lock();

    int max;
    float min=200;
    double distance;
    double x1, y1, z1, x2, y2, z2;

    max=obstacle_msg->get_points_size();

    vector<pair<double,double*> > vect;

    x1=obstacle_msg->points[0].x;
    y1=obstacle_msg->points[0].y;
    z1=obstacle_msg->points[0].z;

    min=sqrt( pow(x1,2) + pow(y1,2) + pow(z1,2));

    // First we read from the PointCloud message the scans of the laser
    // and then update the variable obstacle with the closest point.

    for(int i=1; i<max; i++){
        x2=obstacle_msg->points[i].x;
        y2=obstacle_msg->points[i].y;
        z2=obstacle_msg->points[i].z;

        distance=sqrt( pow(x2,2) + pow(y2,2) + pow(z2,2));
        if(distance<min){
            x1=x2;
            y1=y2;
            z1=z2;
            min=distance;
        }
    }

    this->obstacle[0]=x1;
    this->obstacle[1]=y1;
    this->obstacle[2]=z1;

    lock.unlock();

}

void FeedbackLinearization::ObstacleQueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_obstacle.callAvailable(ros::WallDuration(timeout));
  }
}

void FeedbackLinearization::saturationForce()
{
    if(this->U1_reale >= MAX && this->Eta >0){
        this->U1_reale=MAX;
        this->Eta=0;
    }

    if(this->U1_reale <= MIN && this->Eta <0){
        this->U1_reale=MIN;
        this->Eta=0;
    }

    for(int i=1; i<4; i++ ){
        if(U_tilde[i]>MAX_ROT){
            U_tilde[i]=MAX_ROT;
        }
        if(U_tilde[i]<MIN_ROT){
            U_tilde[i]=MIN_ROT;
        }
    }

}
