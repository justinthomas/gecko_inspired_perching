// Standard C++
#include <math.h>
#include <iostream>

// ROS related
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

// TF
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>

// quadrotor_control
#include <controllers_manager/Transition.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PWMCommand.h>

// Project specific
#include "nano_kontrol2.h"
#include <trajectory.h>

using namespace std;

enum controller_state
{
  ESTOP,
  INIT,
  TAKEOFF,
  PREP_TAKEOFF_TRAJ,
  TAKEOFF_TRAJ,
  HOVER,
  LINE_TRACKER_MIN_JERK,
  LINE_TRACKER_YAW,
  VELOCITY_TRACKER,
  PERCH,
  RECOVER,
  PREP_TRAJ,
  PERCH_TRAJ,
  NONE,
};

// Variables and parameters
double xoff, yoff, zoff, yaw_off, mass_;
bool safety; //, cut_motors_after_traj;
static geometry_msgs::Point home_;
double kR_[3], kOm_[3], corrections_[3];

// Stuff for trajectory
#include <string>
Trajectory perch_traj, takeoff_traj;
static ros::Publisher pub_position_cmd_;
quadrotor_msgs::PositionCommand traj_goal;
void UpdatePerchTrajectory(Trajectory &traj), UpdateTakeoffTrajectory(Trajectory &traj);
static std::string perch_traj_filename, takeoff_traj_filename;
quadrotor_msgs::SO3Command::Ptr so3_command_(new quadrotor_msgs::SO3Command);

// States
static enum controller_state state_ = PERCH; // INIT;

// Publishers & services
static ros::Publisher pub_goal_min_jerk_;
static ros::Publisher pub_goal_distance_;
static ros::Publisher pub_goal_velocity_;
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_goal_yaw_;
static ros::Publisher pub_info_bool_;
static ros::ServiceClient srv_transition_;
static ros::Publisher so3_command_pub_;
static ros::Publisher pub_pwm_command_;

// Quadrotor Pose
static geometry_msgs::Point pos_, perch_pos_;
static geometry_msgs::Vector3 vel_;
static geometry_msgs::Quaternion ori_;
static geometry_msgs::Quaternion imu_q_;
static bool have_odom_ = false;
// static bool imu_info_ = false;

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker_min_jerk("line_tracker/LineTrackerMinJerk");
static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");
static const std::string null_tracker_str("null_tracker/NullTracker");

// Function Prototypes
void hover_in_place();
void hover_at(const geometry_msgs::Point);
void go_home();
void go_to(const quadrotor_msgs::FlatOutputs);
void estop();

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[estop_button])
    estop();

  // cut_motors_after_traj = msg->buttons[7];

  if (msg->buttons[5] == 1)
  {
    // Reset gripper
    quadrotor_msgs::PWMCommand pwm_cmd;
    pwm_cmd.pwm[0] = 0.6;
    pub_pwm_command_.publish(pwm_cmd);
  }
  else if (msg->buttons[6] == 1)
  {
    // Release Gripper
    quadrotor_msgs::PWMCommand pwm_cmd;
    pwm_cmd.pwm[0] = 0.3;
    pub_pwm_command_.publish(pwm_cmd);
  }
  else
  {
    // Don't rotate servo
    quadrotor_msgs::PWMCommand pwm_cmd;
    pwm_cmd.pwm[0] = 0.47;
    pub_pwm_command_.publish(pwm_cmd);
  }

  if(state_ == INIT)
  {
    if (!have_odom_)
    {
      ROS_INFO("Waiting for Odometry!");
      return;
    }

    // Motors on (Rec)
    if(msg->buttons[motors_on_button])
    {
      ROS_INFO("Sending enable motors command");
      std_msgs::Bool motors_cmd;
      motors_cmd.data = true;
      pub_motors_.publish(motors_cmd);
    }

    // Take off (Play)
    if(msg->buttons[play_button])
    {
      state_ = TAKEOFF;
      ROS_INFO("Initiating launch sequence...");

      // home_ has global scope
      home_.x = pos_.x;
      home_.y = pos_.y;
      home_.z = pos_.z + 0.10;
      pub_goal_distance_.publish(home_);
      usleep(100000);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_distance;
      srv_transition_.call(transition_cmd);
    }
    else
      ROS_INFO("Waiting to take off.  Press Rec to enable motors and Play to Take off.");
  }
  else
  {
    // This is executed every time the midi controller changes
    switch(state_)
    {
      case VELOCITY_TRACKER:
        {
          quadrotor_msgs::FlatOutputs goal;
          goal.x = msg->axes[0] * fabs(msg->axes[0]) / 2;
          goal.y = msg->axes[1] * fabs(msg->axes[1]) / 2;
          goal.z = msg->axes[2] * fabs(msg->axes[2]) / 2;
          goal.yaw = msg->axes[3] * fabs(msg->axes[3]) / 2;

          pub_goal_velocity_.publish(goal);
          ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", goal.x, goal.y, goal.z, goal.yaw);
        }
        break;

      default:
        break;
    }

    // Determine whether or not the quad is selected
    bool selected = true;

    // Hover
    if(msg->buttons[hover_button])  // Marker Set
    {
      hover_in_place();
    }
    // Line Tracker
    else if(selected && msg->buttons[line_tracker_button] && (state_ == HOVER || state_ == LINE_TRACKER_MIN_JERK || state_ == TAKEOFF))
    {
      state_ = LINE_TRACKER_MIN_JERK;
      ROS_INFO("Engaging controller: LINE_TRACKER_MIN_JERK");
      geometry_msgs::Point goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = 2*msg->axes[2] + 2.0 + zoff;
      pub_goal_min_jerk_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_min_jerk;
      srv_transition_.call(transition_cmd);
    }
    // Line Tracker Yaw
    else if(selected && msg->buttons[line_tracker_yaw_button] && (state_ == HOVER || state_ == LINE_TRACKER_YAW || state_ == TAKEOFF))
    {
      quadrotor_msgs::FlatOutputs goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = 2*msg->axes[2] + 2.0 + zoff;
      goal.yaw = M_PI * msg->axes[3] + yaw_off;
      go_to(goal);
    }
    // Velocity Tracker
    else if(selected && msg->buttons[velocity_tracker_button] && state_ == HOVER)
    {
      // Note: We do not want to send a goal of 0 if we are
      // already in the velocity tracker controller since it
      // could cause steps in the velocity.

      state_ = VELOCITY_TRACKER;
      ROS_INFO("Engaging controller: VELOCITY_TRACKER");

      quadrotor_msgs::FlatOutputs goal;
      goal.x = 0;
      goal.y = 0;
      goal.z = 0;
      goal.yaw = 0;
      pub_goal_velocity_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = velocity_tracker_str;
      srv_transition_.call(transition_cmd);
    }
    else if(msg->buttons[traj_button] && state_ == HOVER)
    {

      // If there are any errors
      if (!perch_traj.isLoaded())
      {
        hover_in_place();
        ROS_WARN("Couldn't load %s.  Error: %d.  Hovering in place...",
            perch_traj.get_filename().c_str(), perch_traj.get_error_code());
      }
      else
      {
        state_ = PREP_TRAJ;
        ROS_INFO("Loading Trajectory.  state_ == PREP_TRAJ;");

        // Updates traj goal to allow for correct initalization of the trajectory
        perch_traj.set_start_time();
        perch_traj.UpdateGoal(traj_goal);

        quadrotor_msgs::FlatOutputs goal;
        goal.x = traj_goal.position.x;
        goal.y = traj_goal.position.y;
        goal.z = traj_goal.position.z;
        goal.yaw = traj_goal.yaw;

        pub_goal_yaw_.publish(goal);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = line_tracker_yaw;
        srv_transition_.call(transition_cmd);
      }
    }
    else if(msg->buttons[play_button] && state_ == PREP_TRAJ)
    {
      // If we are ready to start the trajectory
      if ( sqrt( pow(traj_goal.position.x + xoff - pos_.x, 2) + pow(traj_goal.position.y + yoff - pos_.y, 2) + pow(traj_goal.position.z + zoff - pos_.z, 2) ) < .03 ||
           sqrt( pow(vel_.x,2) + pow(vel_.y,2) + pow(vel_.z,2) ) < 0.05)
      {
        ROS_INFO("Starting Trajectory");

        state_ = PERCH_TRAJ;

        // Publish the trajectory signal
        std_msgs::Bool traj_on_signal;
        traj_on_signal.data = true;
        pub_info_bool_.publish(traj_on_signal);

        perch_traj.set_start_time();
        perch_traj.UpdateGoal(traj_goal);

        pub_position_cmd_.publish(traj_goal);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = null_tracker_str;
        srv_transition_.call(transition_cmd);
      }
      else
      {
        ROS_WARN("Not ready to start trajectory.");
      }
    }
    else if(state_ == PERCH && msg->buttons[4])
    {
      if (!takeoff_traj.isLoaded())
      {
        ROS_WARN("Couldn't load %s.  Error: %d.",
            takeoff_traj.get_filename().c_str(), takeoff_traj.get_error_code());
      }
      else
      {
        // Prepare for takeoff
        state_ = PREP_TAKEOFF_TRAJ;
        ROS_INFO("state = PREP_TAKEOFF_TRAJ");
        perch_pos_ = pos_;

        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = null_tracker_str;
        srv_transition_.call(transition_cmd);

        ROS_INFO("Sending enable motors command");
        std_msgs::Bool motors_cmd;
        motors_cmd.data = true;
        pub_motors_.publish(motors_cmd);

        takeoff_traj.setOffsets(pos_.x, pos_.y, pos_.z, yaw_off);
        takeoff_traj.set_start_time();
        takeoff_traj.UpdateGoal(traj_goal);
      }
    }
  }
}

void UpdatePerchTrajectory(Trajectory &traj)
{
  if (traj.isCompleted())
  {
    ROS_INFO("Trajectory completed.");

    // Publish that we are exiting the trajectory
    std_msgs::Bool traj_on_signal;
    traj_on_signal.data = false;
    pub_info_bool_.publish(traj_on_signal);

    state_ = PERCH;
    ROS_INFO("Switching to NullTracker");
    controllers_manager::Transition transition_cmd;
    transition_cmd.request.controller = null_tracker_str;
    srv_transition_.call(transition_cmd);

    // Generate the so3 command
    so3_command_->header.stamp = ros::Time::now();
    so3_command_->header.frame_id = "temp_frame_id";
    so3_command_->force.x = 0;
    so3_command_->force.y = 0;
    so3_command_->force.z = 0;
    so3_command_->orientation.x = ori_.x;
    so3_command_->orientation.y = ori_.y;
    so3_command_->orientation.z = ori_.z;
    so3_command_->orientation.w = ori_.w;
    so3_command_->angular_velocity.x = 0;
    so3_command_->angular_velocity.y = 0;
    so3_command_->angular_velocity.z = 0;
    for(int i = 0; i < 3; i++)
    {
      so3_command_->kR[i] = 0;
      so3_command_->kOm[i] = 0;
    }
    so3_command_->aux.current_yaw = 0;
    so3_command_->aux.kf_correction = 0;
    so3_command_->aux.angle_corrections[0] = 0;
    so3_command_->aux.angle_corrections[1] = 0;
    so3_command_->aux.enable_motors = true;
    so3_command_->aux.use_external_yaw = false;
    so3_command_pub_.publish(so3_command_);
  }
  else
  {
    traj.UpdateGoal(traj_goal);
    pub_position_cmd_.publish(traj_goal);
  }
}

void UpdateTakeoffTrajectory(Trajectory &traj)
{
  traj.UpdateGoal(traj_goal);
  pub_position_cmd_.publish(traj_goal);

  if (traj.isCompleted())
  {
    // Publish that we finished the trajectory
    std_msgs::Bool traj_signal;
    traj_signal.data = false;
    pub_info_bool_.publish(traj_signal);
  }
}

/*
static void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_q_ = msg->orientation;
  imu_info_ = true;
}
*/

void estop()
{
  state_ = ESTOP;

  // Publish the E-Stop command
  ROS_WARN("E-STOP");
  std_msgs::Empty estop_cmd;
  pub_estop_.publish(estop_cmd);

  // Disable motors
  ROS_WARN("Disarming motors...");
  std_msgs::Bool motors_cmd;
  motors_cmd.data = false;
  pub_motors_.publish(motors_cmd);

  // Switch to null_tracker
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = null_tracker_str;
  srv_transition_.call(transition_cmd);

  // Publish so3_command to stop motors
  so3_command_->aux.enable_motors = false;
  so3_command_pub_.publish(so3_command_);
}

void hover_at(const geometry_msgs::Point goal)
{
  state_ = HOVER;
  ROS_INFO("Hovering at (%2.2f, %2.2f, %2.2f)", goal.x, goal.y, goal.z);

  pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance;
  srv_transition_.call(transition_cmd);
}

void hover_in_place()
{
  state_ = HOVER;
  ROS_INFO("Hovering in place...");

  geometry_msgs::Point goal;
  goal.x = pos_.x;
  goal.y = pos_.y;
  goal.z = pos_.z;
  pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance;
  srv_transition_.call(transition_cmd);
}

void go_home()
{
  state_ = LINE_TRACKER_MIN_JERK;
  ROS_INFO("Engaging controller: LINE_TRACKER_MIN_JERK");
  pub_goal_min_jerk_.publish(home_);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_min_jerk;
  srv_transition_.call(transition_cmd);
}

void go_to(const quadrotor_msgs::FlatOutputs goal)
{
  state_ = LINE_TRACKER_YAW;
  ROS_INFO("Engaging controller: LINE_TRACKER_YAW");
  pub_goal_yaw_.publish(goal);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_yaw;
  srv_transition_.call(transition_cmd);
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  vel_ = msg->twist.twist.linear;
  ori_ = msg->pose.pose.orientation;

  // Variables maybe needed in switch
  quadrotor_msgs::PositionCommand pos_cmd;

  switch (state_)
  {
    case PERCH_TRAJ:

      UpdatePerchTrajectory(perch_traj);
      break;

    case PERCH:

      so3_command_pub_.publish(so3_command_);

      // If the perch was not successful, then recover
      if (pos_.z < traj_goal.position.z - 0.5 && pos_.x > traj_goal.position.x)
      {
        state_ = RECOVER;

        // Switch to the correct controller
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = null_tracker_str;
        srv_transition_.call(transition_cmd);

        pos_cmd.position.x = pos_.x + 0.5;
        pos_cmd.position.y = pos_.y + 0.2;
        pos_cmd.position.z = pos_.z + 0.5;
        pos_cmd.velocity.x = 0;
        pos_cmd.velocity.y = 0;
        pos_cmd.velocity.z = 0;
        pos_cmd.acceleration.x = 0;
        pos_cmd.acceleration.y = 0;
        pos_cmd.acceleration.z = 0;
        pos_cmd.jerk.x = 0;
        pos_cmd.jerk.y = 0;
        pos_cmd.jerk.z = 0;
        pos_cmd.kx[0] = 3.7 / 3.0;
        pos_cmd.kx[1] = 3.7 / 3.0;
        pos_cmd.kx[2] = 8.0 / 3.0;
        pos_cmd.kv[0] = 2.4 / 3.0;
        pos_cmd.kv[1] = 2.4 / 3.0;
        pos_cmd.kv[2] = 3.0 / 3.0;

        ROS_WARN_THROTTLE(1, "Attempting to recover...");
        pub_position_cmd_.publish(pos_cmd);
      }
      break;

    case RECOVER:
      break;

    case PREP_TAKEOFF_TRAJ:

      // We will check the vertical displacement to determine when to switch to the trajectory
      static double takeoff_ramp = 0;
      if (takeoff_ramp < 1.0)
        takeoff_ramp += 0.001;

      // The position command
      pos_cmd = traj_goal;
      pos_cmd.acceleration.x = traj_goal.acceleration.x * takeoff_ramp;
      pos_cmd.acceleration.y = traj_goal.acceleration.y * takeoff_ramp;
      pos_cmd.acceleration.z = (9.81 + traj_goal.acceleration.z) * takeoff_ramp - 9.81; // kGravity;
      // cout << "pos_cmd.accel: {" << pos_cmd.acceleration.x  << ", " << pos_cmd.acceleration.y << ", " << pos_cmd.acceleration.z << "}" << endl;
      pos_cmd.jerk.x = 0; pos_cmd.jerk.y = 0; pos_cmd.jerk.z = 0;
      pos_cmd.yaw = traj_goal.yaw; pos_cmd.yaw_dot = 0;
      pos_cmd.kx[0] = 0; pos_cmd.kx[1] = 0; pos_cmd.kx[2] = 0;
      pos_cmd.kv[0] = 0; pos_cmd.kv[1] = 0; pos_cmd.kv[2] = 0;
      pub_position_cmd_.publish(pos_cmd);

      // Toggle release
      if (pos_.z < perch_pos_.z - 0.04 || pos_.x > perch_pos_.x + 0.04)
      {
        state_ = TAKEOFF_TRAJ;
        ROS_INFO("state = TAKEOFF_TRAJ");
        takeoff_traj.setOffsets(pos_.x, pos_.y, pos_.z, yaw_off);
        takeoff_traj.set_start_time();

        // Publish that we are starting the trajectory
        std_msgs::Bool traj_on_signal;
        traj_on_signal.data = true;
        pub_info_bool_.publish(traj_on_signal);
      }
      break;

    case TAKEOFF_TRAJ:

      UpdateTakeoffTrajectory(takeoff_traj);
      break;

    default:
      break;
  }

  static tf::Quaternion q;
  static double roll, pitch, yaw;
  tf::quaternionMsgToTF(ori_, q);
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  // TF broadcaster to broadcast the quadrotor frame
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pos_.x, pos_.y, pos_.z) );
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/simulator", "/quadrotor"));

  static tf::Matrix3x3 R;
  R.setEulerYPR(0, pitch, roll);
  R.getRotation(q);
  q.normalize();

  // Position and attitude Safety Catch
  if (safety && (abs(pos_.x) > 2.2 || abs(pos_.y) > 1.8|| pos_.z > 3.5 || pos_.z < 0.2))
  {
    ROS_WARN("Robot has exited safe box. Safety Catch initiated...");
    hover_in_place();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_control");
  ros::NodeHandle n("~");

  // Now, we need to set the formation offsets for this robot
  n.param("offsets/x", xoff, 0.0);
  n.param("offsets/y", yoff, 0.0);
  n.param("offsets/z", zoff, 0.0);
  n.param("offsets/yaw", yaw_off, 0.0);
  /*
  n.param("so3_control/gains/pos/x", kx_[0]);
  n.param("so3_control/gains/pos/y", kx_[1]);
  n.param("so3_control/gains/pos/z", kx_[2]);
  n.param("so3_control/gains/vel/x", kv_[0]);
  n.param("so3_control/gains/vel/y", kv_[1]);
  n.param("so3_control/gains/vel/z", kv_[2]); */
  ROS_INFO("Quad using offsets: {xoff: %2.2f, yoff: %2.2f, zoff: %2.2f, yaw_off: %2.2f}", xoff, yoff, zoff, yaw_off);

  // Attitude gains
  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);
  ROS_INFO("Attitude gains: kR: {%2.2f, %2.2f, %2.2f}, kOm: {%2.2f, %2.2f, %2.2f}", kR_[0], kR_[1], kR_[2], kOm_[0], kOm_[1], kOm_[2]);

  // Corrections
  n.param("corrections/kf", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  // The perching trajectory
  n.param("perch_traj_filename", perch_traj_filename, string("perch_traj.csv"));
  perch_traj.set_filename(perch_traj_filename);
  perch_traj.setOffsets(xoff, yoff, zoff, yaw_off);
  perch_traj.LoadTrajectory();

  // The takeoff trajectory
  n.param("takeoff_traj_filename", takeoff_traj_filename, string("takeoff_traj.csv"));
  takeoff_traj.set_filename(takeoff_traj_filename);
  takeoff_traj.setOffsets(xoff, yoff, zoff, yaw_off);
  takeoff_traj.LoadTrajectory();

  n.param("safety_catch", safety, true);
  n.param("mass", mass_, 0.5);

  // Publishers
  srv_transition_ = n.serviceClient<controllers_manager::Transition>("controllers_manager/transition");
  pub_goal_min_jerk_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_min_jerk/goal", 1);
  pub_goal_distance_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_distance/goal", 1);
  pub_goal_velocity_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/velocity_tracker/vel_cmd_with_yaw", 1);
  pub_goal_yaw_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/line_tracker_yaw/goal", 1);
  pub_position_cmd_ = n.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 1);
  pub_info_bool_ = n.advertise<std_msgs::Bool>("traj_signal", 1);
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);
  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 1);
  pub_pwm_command_ = n.advertise<quadrotor_msgs::PWMCommand>("pwm_cmd", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 10, &odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 10, nanokontrol_cb, ros::TransportHints().tcpNoDelay());

  // Spin
  ros::spin();

  return 0;
}
