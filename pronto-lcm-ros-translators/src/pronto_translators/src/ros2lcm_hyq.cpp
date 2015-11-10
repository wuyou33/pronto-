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
#include <fstream>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/robot_state_t.hpp"
#include "lcmtypes/pronto/joint_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/mav/ins_t.hpp"

#include "ViconDerivator.hpp"

#define MAX_LIDAR_RANGE 60.0


using namespace std;

std::string vicon_frame = "vicon/hyq_green/body";

class App {
public:
    App(ros::NodeHandle& node,
        bool send_ground_truth_,
        bool send_pose_body);
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
    void pose_vicon_cb(const geometry_msgs::TransformStampedConstPtr& msg);

    ros::Subscriber imuSensorSub_;
    void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);

    ros::Subscriber laserScanSub_;
    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
    void publishLidar(const sensor_msgs::LaserScanConstPtr& msg, string channel );

    int64_t last_joint_state_utime_;
    bool verbose_;
    mav::ins_t imu;
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
    imuSensorSub_ = node_.subscribe("/imu/imu", 100, &App::imuSensorCallback, this);

    laserScanSub_ = node_.subscribe("scan", 100, &App::laserScanCallback, this);

    verbose_ = false;
    ROS_INFO("ros2lcm Translator ready");
}

App::~App()  {
    delete vd_;
}


int js_counter = 0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg) {
    if (js_counter % 1000 == 0) {
        ROS_INFO("JNT  [%d]", js_counter );
    }
    js_counter++;

    int n_joints = msg->position.size();

    if (n_joints == 2) {
        ptu_name_ = msg->name;
        ptu_position_ = msg->position;
        ptu_velocity_ = msg->velocity;
        // no efforts supplied
        return;
    }

    pronto::joint_state_t msg_out;
    msg_out.utime = (int64_t) msg->header.stamp.toNSec() / 1000; // from nsec to usec
    msg_out.joint_position.assign(n_joints , std::numeric_limits<int>::min()  );
    msg_out.joint_velocity.assign(n_joints , std::numeric_limits<int>::min()  );
    msg_out.joint_effort.assign(n_joints , std::numeric_limits<int>::min()  );
    msg_out.joint_name = msg->name;
    for (std::vector<int>::size_type i = 0; i < n_joints; i++)  {
        msg_out.joint_position[i] = (float) msg->position[i];
        msg_out.joint_velocity[i] = (float) msg->velocity[i];
        msg_out.joint_effort[i] = (float) msg->effort[i];
    }
    for (std::vector<int>::size_type i = 0; i < ptu_name_.size(); i++)  {
        msg_out.joint_name.push_back( ptu_name_[i]);
        msg_out.joint_position.push_back( (float)ptu_position_[i]);
        msg_out.joint_velocity.push_back( (float)ptu_velocity_[i]);
        msg_out.joint_effort.push_back( (float)ptu_effort_[i]);
    }
    msg_out.num_joints = msg_out.joint_effort.size();
    lcmPublish_.publish("HYQ_STATE", &msg_out);
}


void App::pose_vicon_cb(const geometry_msgs::TransformStampedConstPtr& msg) {
    Eigen::Vector3d pos;
    Eigen::Vector3d vicon_vel;
    Eigen::Affine3d a;
    a.setIdentity();
    a.translation() << msg->transform.translation.x,
                  msg->transform.translation.y,
                  msg->transform.translation.z;

    Eigen::Quaterniond a_q(msg->transform.rotation.w,
                           msg->transform.rotation.x,
                           msg->transform.rotation.y,
                           msg->transform.rotation.z);
    a.rotate(a_q);

    Eigen::Affine3d c = a * bTf;

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
            pos << c.translation().x(), c.translation().y(), c.translation().z();
            vd_->updateMA(msg->header.stamp.toSec(), pos, vicon_vel);
            return;

        } catch (tf::TransformException ex) {
            std::cerr << "ERROR: vicon transform not received!" << std::endl;
            std::cerr << ex.what() << std::endl;
            return;
        }
    } else {

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


void App::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg) {

    imu.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000.0);
    imu.device_time = imu.utime;
    if(imu.gyro[0] == msg->angular_velocity.x &&
            imu.gyro[1] == msg->angular_velocity.y &&
            imu.gyro[2] == msg->angular_velocity.z ) {
        return;
    }
    imu.gyro[0] = msg->angular_velocity.x;
    imu.gyro[1] = msg->angular_velocity.y;
    imu.gyro[2] = msg->angular_velocity.z;

    imu.accel[0] = msg->linear_acceleration.x;
    imu.accel[1] = msg->linear_acceleration.y;
    imu.accel[2] = msg->linear_acceleration.z;

    imu.quat[0] = msg->orientation.w;
    imu.quat[1] = msg->orientation.x;
    imu.quat[2] = msg->orientation.y;
    imu.quat[3] = msg->orientation.z;

    lcmPublish_.publish( ("MICROSTRAIN_INS") , &imu);
}


int scan_counter = 0;
void App::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
    if (scan_counter % 80 == 0) {
        ROS_INFO("LSCAN [%d]", scan_counter );
    }
    scan_counter++;
    publishLidar(msg, "SCAN");
}


void App::publishLidar(const sensor_msgs::LaserScanConstPtr& msg, string channel ) {
    bot_core::planar_lidar_t scan_out;
    scan_out.ranges = msg->ranges;
    for (size_t i = 0; i < scan_out.ranges.size(); i++) {
        if (isnan(scan_out.ranges[i])) {
            scan_out.ranges[i] = MAX_LIDAR_RANGE;
        } else if (isinf(scan_out.ranges[i])) {
            scan_out.ranges[i] = MAX_LIDAR_RANGE;
        }
    }

    scan_out.intensities = msg->intensities;
    scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000);
    scan_out.nranges = msg->ranges.size();
    scan_out.nintensities = msg->intensities.size();
    scan_out.rad0 = msg->angle_min;
    scan_out.radstep = msg->angle_increment;
    lcmPublish_.publish(channel.c_str(), &scan_out);
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "ros2lcm");

    bool send_ground_truth = false;
    bool send_pose_body = false;

    if(argc >= 2) {
        if(strncmp(argv[1], "-b", 2) == 0) {
            send_pose_body = true;
        } else {
            if(argc >= 3 && strncmp(argv[1], "--vicon-frame", 2) == 0) {
                vicon_frame = argv[2];
            }
        }
    }

    ROS_INFO("Vicon frame id: \"%s\"", vicon_frame.c_str());

    if(send_pose_body) {
        ROS_INFO("Sending pose body in translation");
    } else {
        ROS_INFO("Not sending pose body in translation");
    }


    ros::NodeHandle nh;
    new App(nh, send_ground_truth, send_pose_body);

    ros::spin();
    return 0;
}
