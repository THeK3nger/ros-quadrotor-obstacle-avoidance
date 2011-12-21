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

    class FeedbackLinearization : public Controller
    {
        public:
          FeedbackLinearization(Entity *parent);
          virtual ~FeedbackLinearization();
      
        protected:
          virtual void LoadChild(XMLConfigNode *node);
          void SaveChild(std::string &prefix, std::ostream &stream);
          virtual void InitChild();
          void ResetChild();
          virtual void UpdateChild();
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
