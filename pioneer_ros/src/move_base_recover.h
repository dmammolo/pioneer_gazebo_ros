#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include <math.h>

using namespace std ;

class MoveBaseRecover{
  public:
    MoveBaseRecover(ros::NodeHandle) ;
    ~MoveBaseRecover(){}
  private:
    ros::Subscriber subResult ;
    ros::Subscriber subCmdVel ;
    ros::Subscriber subPlan;
    ros::Publisher pubCmdVel ;
    
    ros::Time initialStoppingTime ;
    ros::Time initialRecoveryTime ;
    ros::Time lastLocalPlanTime ;
    ros::Time initialXRecoveryTime;
    bool fGoal ;
    bool fTimer ;
    bool fRecovery ;
    bool backRecovery;
    double thetaThreshold ;
    double recoveryTheta ;
    double recoveryX;
    double timeThreshold ;

    void actionCallback(const actionlib_msgs::GoalStatusArray&) ;
    void cmdVelCallback(const geometry_msgs::Twist&) ;
    void localPlanCallback(const nav_msgs::Path&);
};

MoveBaseRecover::MoveBaseRecover(ros::NodeHandle nh){
  subResult = nh.subscribe("move_base/status", 10, &MoveBaseRecover::actionCallback, this) ;
  subCmdVel = nh.subscribe("controller_cmd_vel", 10, &MoveBaseRecover::cmdVelCallback, this) ;
  subPlan = nh.subscribe("move_base/DWAPlannerROS/local_plan", 10, &MoveBaseRecover::localPlanCallback, this);
  pubCmdVel = nh.advertise<geometry_msgs::Twist>("pioneer/cmd_vel", 10) ;
  
  ros::param::get("move_base/TrajectoryPlannerROS/max_vel_theta",thetaThreshold) ;
  ROS_INFO_STREAM("Recovery behaviour will rotate robot in place at " << thetaThreshold << " rad/s") ;
  timeThreshold = 7.0 ;
  fGoal = false ;
  fTimer = false ;
  fRecovery = false ;
  backRecovery = false;
  lastLocalPlanTime = ros::Time::now();
}

void MoveBaseRecover::actionCallback(const actionlib_msgs::GoalStatusArray& msg){
  if (!msg.status_list.empty()){
    if (msg.status_list[0].status == 1)
      fGoal = true ;
    else
      fGoal = false ;
  }
}

void MoveBaseRecover::localPlanCallback(const nav_msgs::Path& msg){
  lastLocalPlanTime = ros::Time::now();
}

void MoveBaseRecover::cmdVelCallback(const geometry_msgs::Twist& msg){
  geometry_msgs::Twist cmd = msg ;
  if (fRecovery){ // recovery mode
    ros::Duration recoveryDuration = ros::Time::now() - initialRecoveryTime ;
    if (recoveryDuration.toSec() < 1.5) // force 1.5 second turnaround
      cmd.angular.z = recoveryTheta ;
    else { // exit recovery mode
      ROS_INFO("Robot exiting recovery mode...") ;
      fRecovery = false ;
      fTimer = false ;
    }
  }
  else if (backRecovery){ // back Recovery mode
    ros::Duration recoveryDuration = ros::Time::now() - initialXRecoveryTime ;
    if (recoveryDuration.toSec() < 2.0) // force 2.0 second turnaround
      cmd.linear.x = recoveryX ;
    else { // exit recovery mode
      ROS_INFO("Robot exiting recovery mode...") ;
      backRecovery = false ;
      fTimer = false ;
    }
  }
  else if (fGoal){
    if (fabs(msg.linear.x) < 0.01 && fabs(msg.angular.z) < thetaThreshold / 3){
      if (!fTimer){
        initialStoppingTime = ros::Time::now() ;
        lastLocalPlanTime = ros::Time::now(); // reset time, as in goal region no local Plan is recived
        fTimer = true ;
      }
      else{
        ros::Duration stoppedTime = ros::Time::now() - initialStoppingTime ;
        ros::Duration lastLocalPlanDiff = ros::Time::now() - lastLocalPlanTime;

        if (lastLocalPlanDiff.toSec() > timeThreshold - 1){
          geometry_msgs::Twist cmd = msg;
          ROS_INFO("Overriding move_base command velocities to unstick robot in recovery mode... moving backwards...") ;
          backRecovery = true;
          recoveryX = -0.15;
          cmd.linear.x = recoveryX;
          initialXRecoveryTime = ros::Time::now();
        }
        else if (stoppedTime.toSec() > timeThreshold){
          geometry_msgs::Twist cmd = msg ;
          ROS_INFO("Overriding move_base command velocities to unstick robot in recovery mode...") ;
          fRecovery = true ; // enter recovery mode
          if (msg.angular.z < 0.0)
            recoveryTheta = -thetaThreshold ;
          else
            recoveryTheta = thetaThreshold ;
          cmd.angular.z = recoveryTheta ;
          initialRecoveryTime = ros::Time::now() ;
        }
      }
    }
    else
      fTimer = false ;
  }
  pubCmdVel.publish(cmd) ;
}
