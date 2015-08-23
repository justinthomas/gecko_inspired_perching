// Standard C++
#include <memory>
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
#include <quadrotor_msgs/SO3Command.h>
#include <mav_manager/manager.h>
#include <quadrotor_msgs/PWMCommand.h>

// Project specific
#include "nano_kontrol2.h"
#include <trajectory.h>

using namespace std;

std::shared_ptr<MAVManager> mav;

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
static enum controller_state state_ = INIT;

// Publishers & services
static ros::Publisher pub_info_bool_;
static ros::Publisher so3_command_pub_;
static ros::Publisher pub_pwm_command_;

// Quadrotor Pose
static geometry_msgs::Point pos_, perch_pos_;
static geometry_msgs::Quaternion ori_;
static geometry_msgs::Quaternion imu_q_;
static bool have_odom_ = false;
// static bool imu_info_ = false;

// Perch plate
static bool pp_have_odom_ = false;
static tf::Vector3 pp_pos_;
static tf::Vector3 pp_b3_;

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[estop_button])
    mav->estop();

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
    if (!mav->have_recent_odom())
    {
      ROS_INFO("Waiting for Odometry!");
      return;
    }

    // Motors on (Rec)
    if(msg->buttons[motors_on_button])
      mav->motors(true);

    // Take off (Play)
    if(msg->buttons[play_button])
    {
      state_ = TAKEOFF;
      mav->takeoff();
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
          double x = msg->axes[0] * fabs(msg->axes[0]) / 2;
          double y = msg->axes[1] * fabs(msg->axes[1]) / 2;
          double z = msg->axes[2] * fabs(msg->axes[2]) / 2;
          double yaw = msg->axes[3] * fabs(msg->axes[3]) / 2;

          mav->setDesVelWorld(x, y, z, yaw);
          ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", x, y, z, yaw);
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
      if (mav->hover())
        state_ = HOVER;
    }
    else if(selected && msg->buttons[3] && (state_ == HOVER || state_ == LINE_TRACKER_MIN_JERK))
    {
      if (mav->goHome())
        state_ = LINE_TRACKER_MIN_JERK;
    }
    // Line Tracker
    else if(selected && msg->buttons[line_tracker_button] && (state_ == HOVER || state_ == LINE_TRACKER_MIN_JERK || state_ == TAKEOFF))
    {
      state_ = LINE_TRACKER_MIN_JERK;
      double x = 2*msg->axes[0] + xoff;
      double y = 2*msg->axes[1] + yoff;
      double z = 2*msg->axes[2] + 2.0 + zoff;
      mav->goTo(x, y, z, mav->yaw());
    }
    // Line Tracker Yaw
    else if(selected && msg->buttons[line_tracker_yaw_button] && (state_ == HOVER || state_ == LINE_TRACKER_YAW || state_ == TAKEOFF))
    {
      double x = 2*msg->axes[0] + xoff;
      double y = 2*msg->axes[1] + yoff;
      double z = 2*msg->axes[2] + 2.0 + zoff;
      double yaw = M_PI * msg->axes[3] + yaw_off;
      mav->goTo(x, y, z, yaw);
    }
    // Velocity Tracker
    else if(selected && msg->buttons[velocity_tracker_button] && state_ == HOVER)
    {
      // Note: We do not want to send a goal of 0 if we are
      // already in the velocity tracker controller since it
      // could cause steps in the velocity.

      state_ = VELOCITY_TRACKER;
      mav->setDesVelWorld(0, 0, 0, 0);
    }
    else if(msg->buttons[traj_button] && state_ == HOVER)
    {
      // If there are any errors
      if (!perch_traj.isLoaded())
      {
        if (mav->hover())
          state_ = HOVER;

        ROS_WARN("Couldn't load %s.  Error: %d.  Hovering in place...",
            perch_traj.get_filename().c_str(), perch_traj.get_error_code());
      }
      else
      {
        state_ = PREP_TRAJ;
        ROS_INFO("state_ == PREP_TRAJ;");

        // Updates traj goal to allow for correct initalization of the trajectory
        pp_pos_ = pp_pos_ - 0.08 * pp_b3_; // Shift to consider the offset of the pads from the center of the robot
        perch_traj.setOffsets(pp_pos_[0], pp_pos_[1], pp_pos_[2], yaw_off);
        perch_traj.set_start_time();
        perch_traj.UpdateGoal(traj_goal);

        mav->goTo(
            traj_goal.position.x,
            traj_goal.position.y,
            traj_goal.position.z,
            traj_goal.yaw);
      }
    }
    else if(msg->buttons[play_button] && state_ == PREP_TRAJ)
    {
      Eigen::Vector3d pos = mav->pos();
      Eigen::Vector3d vel = mav->vel();
      // If we are ready to start the trajectory
      if ( sqrt( pow(traj_goal.position.x + xoff - pos[0], 2)
            + pow(traj_goal.position.y + yoff - pos[1], 2)
            + pow(traj_goal.position.z + zoff - pos[2], 2) ) < .03 ||
           sqrt( pow(vel[0],2) + pow(vel[1],2) + pow(vel[2],2) ) < 0.05)
      {
        ROS_INFO("Starting Trajectory");

        state_ = PERCH_TRAJ;

        // Publish the trajectory signal
        std_msgs::Bool traj_on_signal;
        traj_on_signal.data = true;
        pub_info_bool_.publish(traj_on_signal);

        perch_traj.set_start_time();
        perch_traj.UpdateGoal(traj_goal);

        mav->setPositionCommand(traj_goal);
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

        mav->motors(true);

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
    mav->useNullTracker();

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
    mav->setPositionCommand(traj_goal);
  }
}

void UpdateTakeoffTrajectory(Trajectory &traj)
{
  traj.UpdateGoal(traj_goal);
  mav->setPositionCommand(traj_goal);

  if (traj.isCompleted())
  {
    // Publish that we finished the trajectory
    std_msgs::Bool traj_signal;
    traj_signal.data = false;
    pub_info_bool_.publish(traj_signal);
  }
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
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

        mav->hover();

        ROS_WARN_THROTTLE(1, "Attempting to recover...");
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
      pos_cmd.yaw = mav->yaw(); pos_cmd.yaw_dot = 0;
      pos_cmd.kx[0] = 0; pos_cmd.kx[1] = 0; pos_cmd.kx[2] = 0;
      pos_cmd.kv[0] = 0; pos_cmd.kv[1] = 0; pos_cmd.kv[2] = 0;
      mav->setPositionCommand(pos_cmd);

      // Toggle release
      if (pos_.z < perch_pos_.z - 0.05 || pos_.x > perch_pos_.x + 0.05)
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
}

void perch_plate_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  pp_have_odom_ = true;

  pp_pos_[0] = msg->pose.pose.position.x;
  pp_pos_[1] = msg->pose.pose.position.y;
  pp_pos_[2] = msg->pose.pose.position.z;

  tf::Quaternion odom_q = tf::Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  tf::Matrix3x3 R(odom_q);
  pp_b3_ = R.getColumn(2);

  // cout << pp_b3_[0] << ", " << pp_b3_[1] << ", " << pp_b3_[2] << endl;

  // yaw_ = tf::getYaw(msg->pose.pose.orientation);
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
  pub_info_bool_ = n.advertise<std_msgs::Bool>("traj_signal", 1);
  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 1);
  pub_pwm_command_ = n.advertise<quadrotor_msgs::PWMCommand>("pwm_cmd", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 10, &odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_perch_plate = n.subscribe("/PerchPlate/odom", 10, &perch_plate_odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 10, nanokontrol_cb, ros::TransportHints().tcpNoDelay());

  // MAVManager stuff
  mav.reset(new MAVManager());
  mav->set_need_imu(false);
  mav->set_use_attitude_safety_catch(false);

  // Spin
  ros::spin();

  return 0;
}
