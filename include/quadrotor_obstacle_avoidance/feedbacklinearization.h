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
#ifndef FEEDBACKLINEARIZATION_HH
#define FEEDBACKLINEARIZATION_HH

#include <map>
#include <vector>
#include <utility>

#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace std;

namespace gazebo
{
class Joint;
class Entity;

    /**
      This is an implementation of a Gazeebo Controller for a Quadrotor using
      <b>Feedback Linearization</b> for controll and <b>Artificial Potential
      Field</b> for autonomous navigation task.

      Feedback Linarization is a control tecnique that transform a non-linear
      system x'(t) = f(t) + g(t)U(t) in a linear system z'(t) = Az(t) + Bv(t)
      through a particular input U(t).

      Instead Artificial Potential Field is a navigation mathod based on an
      artificial force field computed by sensors inputs and applied to the
      robot.
    **/
    class FeedbackLinearization : public Controller
    {
        public:
          /**
            Constructor
          */
          FeedbackLinearization(Entity *parent);
          virtual ~FeedbackLinearization();
      
        protected:
          /* GAZEBO STANDARD CONTROLLER METHODS */
          /**
            This code is executed when the controlled node is loaded.
          **/
          virtual void LoadChild(XMLConfigNode *node);

          /**
            This code is ecexuted when the controlled node has to be saved.
          **/
          void SaveChild(std::string &prefix, std::ostream &stream);

          /**
            This code is executed after that the controlled node is loaded and
            has to be initialized. It's very similar to LoadChild method.
          **/
          virtual void InitChild();

          /**
            This code is executed on Gazebo restart signal. After the execution
            of this piece of code the node should be restored to the initial
            configuration.
          **/
          void ResetChild();

          /**
            This code is executed every time that the controlled node has to be
            updated. The update rate is specified in the Gazebo world file.
          **/
          virtual void UpdateChild();

          /**
            This code is executed when the controlled node has to be destroyed.
          **/
          virtual void FiniChild();
      
        private:
            Model *parent_;
            ros::NodeHandle* rosnode_;
            
            Body* body_main;
            
            ros::Subscriber sub_goal;
            ros::Subscriber sub_obstacle;
            ros::Subscriber sub_lookat;
            boost::mutex lock;
            ros::CallbackQueue queue_goal;
            ros::CallbackQueue queue_obstacle;
            ros::CallbackQueue queue_lookat;
            boost::thread* goal_callback_queue_thread_;
            boost::thread* obstacle_callback_queue_thread_;
            boost::thread* lookat_callback_queue_thread_;
            
            /*
              QUADROTOR STATE
             */
            double* X;
            double* X_prev;
            double* X_d;
            double* X_d_prev;
            double* X_dd_prev;
            double* U_tilde;
            double  Eta;
            double  U1_reale;
            double* goal;
            double* obstacle;
            double* look_at;
            FILE* id;
            
            void goalCallback(const geometry_msgs::Point::ConstPtr& goal_msg);
            void obstacleCallback(const sensor_msgs::PointCloud::ConstPtr& obstacle_msg);
            void lookatCallback(const geometry_msgs::Point::ConstPtr& lookat_msg);
            void GoalQueueThread();
            void ObstacleQueueThread();
            void LookatQueueThread();
            void saturationForce();
    };

}

#endif
