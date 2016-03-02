// Selective ros2lcm translator
// mfallon
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/robot_state_t.hpp"
#include "lcmtypes/pronto/joint_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/mav/ins_t.hpp"

using namespace std;

class App{
public:
  App(ros::NodeHandle node_, bool send_ground_truth_);
  ~App();

private:
    Eigen::Affine3d bTf; // Transform between viconplate-to-body
    bool first_vicon_transform;
    bool send_ground_truth_; // publish control msgs to LCM
    bool send_pose_body_;
    lcm::LCM lcmPublish_ ;
    ros::NodeHandle node_;

    // Atlas Joints and FT sensor
    ros::Subscriber  joint_states_sub_;
    void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);
    std::vector<double> ptu_position_, ptu_velocity_, ptu_effort_;
    std::vector<std::string> ptu_name_;

    // The position and orientation from a vicon system:
    ros::Subscriber pose_vicon_sub_;
    ros::Subscriber pose_sim_ground_truth;
    void pose_vicon_cb(const geometry_msgs::TransformStampedConstPtr& msg);

    ros::Subscriber imuSensorSub_;
    void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);

    ros::Subscriber laserScanSub_;
    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);

    void simGroundTruthCallback(const nav_msgs::OdometryConstPtr& msg);

    void publishLidar(const sensor_msgs::LaserScanConstPtr& msg, string channel );

    int64_t last_joint_state_utime_;
    bool verbose_;
    mav::ins_t imu;
    bot_core::pose_t pose_ground_truth;
    bot_core::rigid_transform_t transf_ground_truth;

    tf::TransformListener listener_;
    ViconDerivator* vd_;
};

App::App(ros::NodeHandle& node,
         bool send_ground_truth,
         bool send_pose_body) :

    send_ground_truth_(send_ground_truth),
    node_(node),
    send_pose_body_(send_pose_body),
    first_vicon_transform(true),
    bTf(Eigen::Affine3d::Identity()),
    vd_(new ViconDerivator(5)) {

    ROS_INFO("Initializing Translator");
    if(!lcmPublish_.good()) {
        std::cerr << "ERROR: lcm is not good()" << std::endl;
    }

    for(int i = 0; i < 4; i++) {
        imu.quat[i] = 0;
        if(i < 3) {
            imu.accel[i] = 0;
            imu.gyro[i] = 0;
            imu.mag[i] = 0;
        }
    }
    imu.pressure = 0;
    imu.rel_alt = 0;

    joint_states_sub_ = node_.subscribe(string("joint_states"), 100, &App::joint_states_cb, this);
    ptu_name_ = {"ptu_pan", "ptu_tilt"};
    ptu_position_ = {0.0, 0.0};
    ptu_velocity_ = {0.0, 0.0};
    ptu_effort_ = {0.0, 0.0};

    pose_vicon_sub_ = node_.subscribe(vicon_frame, 100, &App::pose_vicon_cb, this);
    pose_vicon_sub_ = node_.subscribe("/hyq2max/ground_truth_odom", 100, &App::simGroundTruthCallback, this);
    imuSensorSub_ = node_.subscribe("/imu/imu", 100, &App::imuSensorCallback, this);

    laserScanSub_ = node_.subscribe("scan", 100, &App::laserScanCallback, this);

    verbose_ = false;
    ROS_INFO("ros2lcm Translator ready");
}

  joint_states_sub_ = node_.subscribe(string("joint_states"), 100, &App::joint_states_cb,this);
  ptu_name_ = {"ptu_pan", "ptu_tilt"};
  ptu_position_ = {0.0,0.0};
  ptu_velocity_ = {0.0,0.0};
  ptu_effort_ = {0.0,0.0};

void App::simGroundTruthCallback(const nav_msgs::OdometryConstPtr &msg) {
    pose_ground_truth.utime = msg->header.stamp.toNSec() / 1000;

    Eigen::Affine3d world_to_base = Eigen::Affine3d::Identity();
    Eigen::Quaterniond msg_orient(msg->pose.pose.orientation.w,
                                  msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y,
                                  msg->pose.pose.orientation.z);
    world_to_base.rotate(msg_orient);

    pose_ground_truth.pos[0] = msg->pose.pose.position.x;
    pose_ground_truth.pos[1] = msg->pose.pose.position.y;
    pose_ground_truth.pos[2] = msg->pose.pose.position.z;

    pose_ground_truth.orientation[0] = msg->pose.pose.orientation.w;
    pose_ground_truth.orientation[1] = msg->pose.pose.orientation.x;
    pose_ground_truth.orientation[2] = msg->pose.pose.orientation.y;
    pose_ground_truth.orientation[3] = msg->pose.pose.orientation.z;

    // Velocity must be in the base frame, but is provided in the world frame
    Eigen::Vector3d vel;
    vel << msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;
    vel = world_to_base.inverse() * vel;

    pose_ground_truth.vel[0] = vel(0);
    pose_ground_truth.vel[1] = vel(1);
    pose_ground_truth.vel[2] = vel(2);

    pose_ground_truth.rotation_rate[0] = msg->twist.twist.angular.x;
    pose_ground_truth.rotation_rate[1] = msg->twist.twist.angular.y;
    pose_ground_truth.rotation_rate[2] = msg->twist.twist.angular.z;

    lcmPublish_.publish("POSE_GROUND_TRUTH", &pose_ground_truth);

    transf_ground_truth.utime = pose_ground_truth.utime;
    transf_ground_truth.trans[0] = msg->pose.pose.position.x;
    transf_ground_truth.trans[1] = msg->pose.pose.position.y;
    transf_ground_truth.trans[2] = msg->pose.pose.position.z;

    transf_ground_truth.quat[0] = msg->pose.pose.orientation.w;
    transf_ground_truth.quat[1] = msg->pose.pose.orientation.x;
    transf_ground_truth.quat[2] = msg->pose.pose.orientation.y;
    transf_ground_truth.quat[3] = msg->pose.pose.orientation.z;


    lcmPublish_.publish("ODOM_GROUND_TRUTH", &transf_ground_truth);
}


  verbose_ = false;
};

void App::pose_vicon_cb(const geometry_msgs::TransformStampedConstPtr& msg) {
    Eigen::Vector3d pos;
    Eigen::Vector3d vicon_vel;
    Eigen::Affine3d a = Eigen::Affine3d::Identity();


    a.translation() << msg->transform.translation.x,
                  msg->transform.translation.y,
                  msg->transform.translation.z;

    Eigen::Quaterniond a_q(msg->transform.rotation.w,
                           msg->transform.rotation.x,
                           msg->transform.rotation.y,
                           msg->transform.rotation.z);
    a.rotate(a_q);

    if(first_vicon_transform) {
        tf::StampedTransform transform;
        try {
            listener_.waitForTransform(vicon_frame,
                                       "base_link",
                                       ros::Time(0),
                                       ros::Duration(10.0));

            listener_.lookupTransform(vicon_frame,
                                      "base_link",
                                      ros::Time(0),
                                      transform);

            tf::transformTFToEigen(transform, bTf);
            ROS_INFO("First Vicon-to-base transform captured:");
            std::cout << bTf.matrix() << std::endl;
            first_vicon_transform = false;
            Eigen::Affine3d c = a * bTf;
            pos << c.translation().x(), c.translation().y(), c.translation().z();

            vd_->updateMA(msg->header.stamp.toSec(), pos, vicon_vel);
            return;

        } catch (tf::TransformException ex) {
            std::cerr << "ERROR: vicon transform not received!" << std::endl;
            std::cerr << ex.what() << std::endl;
            return;
        }
    } else {
        Eigen::Affine3d c = a * bTf;
        bot_core::rigid_transform_t pose_msg;
        pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000);
        pose_msg.trans[0] = msg->transform.translation.x;
        pose_msg.trans[1] = msg->transform.translation.y;
        pose_msg.trans[2] = msg->transform.translation.z;
        pose_msg.quat[0] =  msg->transform.rotation.w;
        pose_msg.quat[1] =  msg->transform.rotation.x;
        pose_msg.quat[2] =  msg->transform.rotation.y;
        pose_msg.quat[3] =  msg->transform.rotation.z;
        lcmPublish_.publish("VICONSYSTEM_TO_LOCAL", &pose_msg);

        bot_core::pose_t pose_msg3;

        /* getting the estimated velocity */
        pos << c.translation().x(), c.translation().y(), c.translation().z();
        vd_->updateMA(msg->header.stamp.toSec(), pos, vicon_vel);

        Eigen::Affine3d world_to_base = Eigen::Affine3d::Identity();
        world_to_base.rotate(c.rotation());


        // Velocity is expressed in the world frame.
        //We rotate so its in base frame
        vicon_vel = world_to_base.inverse() * vicon_vel;

        pose_msg3.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000);
        pose_msg3.pos[0] = c.translation().x();
        pose_msg3.pos[1] = c.translation().y();
        pose_msg3.pos[2] = c.translation().z();
        pose_msg3.vel[0] = vicon_vel(0);
        pose_msg3.vel[1] = vicon_vel(1);
        pose_msg3.vel[2] = vicon_vel(2);
        pose_msg3.accel[0] = 0;
        pose_msg3.accel[1] = 0;
        pose_msg3.accel[2] = 0;

        Eigen::Quaterniond c_q = Eigen::Quaterniond(c.rotation());
        pose_msg3.orientation[0] =  c_q.w();
        pose_msg3.orientation[1] =  c_q.x();
        pose_msg3.orientation[2] =  c_q.y();
        pose_msg3.orientation[3] =  c_q.z();

        pose_msg3.rotation_rate[0] = 0;
        pose_msg3.rotation_rate[1] = 0;
        pose_msg3.rotation_rate[2] = 0;

        if(send_pose_body_) {
            lcmPublish_.publish("POSE_BODY", &pose_msg3);
        }
        lcmPublish_.publish("POSE_VICON", &pose_msg3);

        bot_core::rigid_transform_t pose_msg4;
        pose_msg4.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000);
        pose_msg4.trans[0] = c.translation().x();
        pose_msg4.trans[1] = c.translation().y();
        pose_msg4.trans[2] = c.translation().z();
        pose_msg4.quat[0] =  c_q.w();
        pose_msg4.quat[1] =  c_q.x();
        pose_msg4.quat[2] =  c_q.y();
        pose_msg4.quat[3] =  c_q.z();
        lcmPublish_.publish("VICON_TO_LOCAL", &pose_msg4);
    }
}


int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  if (js_counter%1000 ==0){
    ROS_ERROR("JNT  [%d]", js_counter );
  }
  js_counter++;

  int n_joints = msg->position.size();

  if (n_joints==2){
    ptu_name_ = msg->name;
    ptu_position_ = msg->position;
    ptu_velocity_ = msg->velocity;
    // no efforts supplied
    return;
  }



  pronto::joint_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  
  msg_out.joint_position.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_velocity.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_effort.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_name = msg->name;
  for (std::vector<int>::size_type i = 0; i < n_joints; i++)  {
    msg_out.joint_position[i] =(float) msg->position[i];
    msg_out.joint_velocity[i] =(float) msg->velocity[i];
    msg_out.joint_effort[i] =(float) msg->effort[i];
  }
  for (std::vector<int>::size_type i = 0; i < ptu_name_.size(); i++)  {
    msg_out.joint_name.push_back( ptu_name_[i]);
    msg_out.joint_position.push_back( (float)ptu_position_[i]);
    msg_out.joint_velocity.push_back( (float)ptu_velocity_[i]);
    msg_out.joint_effort.push_back( (float)ptu_effort_[i]);
  }
  msg_out.num_joints = msg_out.joint_effort.size();
  lcm_publish_.publish("HYQ_STATE", &msg_out);


  if (1==0){ 
  pronto::robot_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  
  msg_out.joint_position.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_velocity.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_effort.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_name = msg->name;
  for (std::vector<int>::size_type i = 0; i < n_joints; i++)  {
    msg_out.joint_position[i] =(float) msg->position[i];
    msg_out.joint_velocity[i] =(float) msg->velocity[i];
    msg_out.joint_effort[i] =(float) msg->effort[i];
  }
  for (std::vector<int>::size_type i = 0; i < ptu_name_.size(); i++)  {
    msg_out.joint_name.push_back( ptu_name_[i]);
    msg_out.joint_position.push_back( (float)ptu_position_[i]);
    msg_out.joint_velocity.push_back( (float)ptu_velocity_[i]);
    msg_out.joint_effort.push_back( (float)ptu_effort_[i]);
  }
  msg_out.num_joints = msg_out.joint_effort.size();
  lcm_publish_.publish("EST_ROBOT_STATE", &msg_out);
  }


}


int gt_counter =0;
void App::pose_bdi_cb(const nav_msgs::OdometryConstPtr& msg){
  if (gt_counter%1000 ==0){
    ROS_ERROR("BDI  [%d]", gt_counter );
  }  
  gt_counter++;

  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);

  if (verbose_)
    std::cout <<"                                                            " << pose_msg.utime << " bdi\n";

  pose_msg.pos[0] = msg->pose.pose.position.x;
  pose_msg.pos[1] = msg->pose.pose.position.y;
  pose_msg.pos[2] = msg->pose.pose.position.z;
  // what about orientation in imu msg?
  pose_msg.orientation[0] =  msg->pose.pose.orientation.w;
  pose_msg.orientation[1] =  msg->pose.pose.orientation.x;
  pose_msg.orientation[2] =  msg->pose.pose.orientation.y;
  pose_msg.orientation[3] =  msg->pose.pose.orientation.z;

  // This transformation is NOT correct for Trooper
  // April 2014: added conversion to body frame - so that both rates are in body frame
  Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z ));
  Eigen::Vector3d lin_body_vel  = R*Eigen::Vector3d ( msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                                      msg->twist.twist.linear.z );
  pose_msg.vel[0] = lin_body_vel[0];
  pose_msg.vel[1] = lin_body_vel[1];
  pose_msg.vel[2] = lin_body_vel[2];


  // this is the body frame rate
  pose_msg.rotation_rate[0] = msg->twist.twist.angular.x;
  pose_msg.rotation_rate[1] = msg->twist.twist.angular.y;
  pose_msg.rotation_rate[2] = msg->twist.twist.angular.z;
  // prefer to take all the info from one source
//  pose_msg.rotation_rate[0] = imu_msg_.angular_velocity.x;
//  pose_msg.rotation_rate[1] = imu_msg_.angular_velocity.y;
//  pose_msg.rotation_rate[2] = imu_msg_.angular_velocity.z;
  
  // Frame?
  //pose_msg.accel[0] = imu_msg_.linear_acceleration.x;
  //pose_msg.accel[1] = imu_msg_.linear_acceleration.y;
  //pose_msg.accel[2] = imu_msg_.linear_acceleration.z;

  lcm_publish_.publish("POSE_BDI", &pose_msg);   
  // lcm_publish_.publish("POSE_BODY", &pose_msg);    // for now
}


void App::pose_vicon_cb(const geometry_msgs::TransformStampedConstPtr& msg){

  Eigen::Matrix4d bTf;
  bTf <<        0.00000,   0.00000,  1.00000,  -0.26100,
        0.00000,   -1.00000,   0.00000,   0.19100,
        1.00000,   0.00000,   0.00000,  -0.44300,
        0.00000,  0.00000,   0.00000,   1.00000;
  Eigen::Isometry3d b(bTf);

  Eigen::Isometry3d a;
  a.setIdentity();
  a.translation() << msg->transform.translation.x,msg->transform.translation.y,msg->transform.translation.z;
  Eigen::Quaterniond a_q(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
  a.rotate(a_q);

  Eigen::Isometry3d c = a*b;

  /*
  bot_core::pose_t pose_msg2;
  pose_msg2.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  pose_msg2.pos[0] = b.translation().x();
  pose_msg2.pos[1] = b.translation().y();
  pose_msg2.pos[2] = b.translation().z();
  Eigen::Quaterniond q_b = Eigen::Quaterniond(b.rotation());
  pose_msg2.orientation[0] =  q_b.w();
  pose_msg2.orientation[1] =  q_b.x();
  pose_msg2.orientation[2] =  q_b.y();
  pose_msg2.orientation[3] =  q_b.z();
  lcm_publish_.publish("POSE_XXX", &pose_msg2);
  */


  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  pose_msg.pos[0] = msg->transform.translation.x;
  pose_msg.pos[1] = msg->transform.translation.y;
  pose_msg.pos[2] = msg->transform.translation.z;
  pose_msg.orientation[0] =  msg->transform.rotation.w;
  pose_msg.orientation[1] =  msg->transform.rotation.x;
  pose_msg.orientation[2] =  msg->transform.rotation.y;
  pose_msg.orientation[3] =  msg->transform.rotation.z;
  lcm_publish_.publish("POSE_VICON", &pose_msg);


  bot_core::pose_t pose_msg3;
  pose_msg3.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  pose_msg3.pos[0] = c.translation().x();
  pose_msg3.pos[1] = c.translation().y();
  pose_msg3.pos[2] = c.translation().z();
  Eigen::Quaterniond c_q = Eigen::Quaterniond(c.rotation());
  pose_msg3.orientation[0] =  c_q.w();
  pose_msg3.orientation[1] =  c_q.x();
  pose_msg3.orientation[2] =  c_q.y();
  pose_msg3.orientation[3] =  c_q.z();
  lcm_publish_.publish("POSE_BODY", &pose_msg3);


}




void App::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg){

  mav::ins_t imu;
  imu.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  imu.device_time = imu.utime;
  imu.gyro[0] = msg->angular_velocity.x;
  imu.gyro[1] = msg->angular_velocity.y;
  imu.gyro[2] = msg->angular_velocity.z;
  imu.mag[0] = 0;
  imu.mag[1] = 0;
  imu.mag[2] = 0;
  imu.accel[0] = msg->linear_acceleration.x;
  imu.accel[1] = msg->linear_acceleration.y;
  imu.accel[2] = msg->linear_acceleration.z;
  imu.quat[0] = msg->orientation.w;
  imu.quat[1] = msg->orientation.x;
  imu.quat[2] = msg->orientation.y;
  imu.quat[2] = msg->orientation.z;
  imu.pressure = 0;
  imu.rel_alt = 0;

  lcm_publish_.publish( ("MICROSTRAIN_INS") , &imu);
}





int main(int argc, char **argv){
  bool send_ground_truth = false;  

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh, send_ground_truth);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("ROS2LCM Translator Ready");
  ros::spin();
  return 0;
}
