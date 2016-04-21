// ros2lcm translator for velodyne
// mcamurri

#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <velodyne_pointcloud/VelodyneConfigConfig.h>
#include <rawdata.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

class App {
public:
    App(ros::NodeHandle& node_,
        ros::NodeHandle& private_node_,
        int modulo,
        float distance, bool use_raw_data);
    ~App();

private:
    lcm::LCM lcmPublish_ ;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>::Ptr sor;
    pcl::RandomSample<pcl::PointXYZ>::Ptr rs;

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_2;

    ros::Subscriber velodyne_pointcloud_sub_;
    ros::Subscriber velodyne_scan_sub_;

    float downsample_factor_;
    int modulo_;
    float distance_;
    std::string velodyne_topic_;

    void velodyne_pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void velodyne_scan_cb(const velodyne_msgs::VelodyneScanConstPtr& msg);

    std::shared_ptr<velodyne_rawdata::RawData> data_;
};

App::App(ros::NodeHandle &node_,
         ros::NodeHandle &private_node_,
         int modulo,
         float distance,
         bool use_raw_data) :
    temp_cloud_1(new pcl::PointCloud<pcl::PointXYZ>()),
    temp_cloud_2(new pcl::PointCloud<pcl::PointXYZ>()),
    sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>()),
    rs(new pcl::RandomSample<pcl::PointXYZ>()),
    modulo_(modulo),
    distance_(distance),
    data_(new velodyne_rawdata::RawData()) {

    if(use_raw_data) {
        std::stringstream ss;
        ss << std::getenv("PRONTO_ROOT") << "/pronto-lcm-ros-translators/"
           "src/pronto_translators/src/32db.yaml";

        private_node_.setParam("calibration", ss.str());
        data_->setup(private_node_);

        // we want everything
        data_->setParameters(2.0, // min range 2 meters
                             7.0, // max range 7 meters
                             // we skip 45 deg in the back, which is the front for
                             // the velodyne because it's rotated by 180 deg from
                             // the robot frame
                             M_PI * 1.0 / 8.0,
                             M_PI * 15.0 / 8.0 );

        // subscribe to VelodyneScan packets
        std::string velodyne_scan_topic = "velodyne_packets";
        velodyne_scan_sub_ = node_.subscribe(
                                 velodyne_scan_topic, 10,
                                 &App::velodyne_scan_cb, (App *) this,
                                 ros::TransportHints().tcpNoDelay(true));
    } else {
        std::string pointcloudTopic = "velodyne_points";
        std::cout << "Subscribing to " << pointcloudTopic << std::endl;
        velodyne_pointcloud_sub_ = node_.subscribe(
                                       pointcloudTopic, 1,
                                       &App::velodyne_pointcloud_cb, this);
    }

    ROS_INFO("Initializing Velodyne Translator");
    if(!lcmPublish_.good()) {
        std::cerr << "ERROR: lcm is not good()" << std::endl;
    }
}

App::~App()  {
}

void App::velodyne_pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    bot_core::pointcloud2_t msg_velo;
    bot_core::pointcloud_t msg_velo_sub_;

    msg_velo_sub_.channel_names = {};
    msg_velo_sub_.frame_id = "VELODYNE";
    msg_velo_sub_.seq = msg->header.seq;
    msg_velo_sub_.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000.0);
    msg_velo_sub_.n_channels = 0;

    pcl::fromROSMsg(*msg, *cloud_in);

    sor->setInputCloud(cloud_in);
    sor->setMeanK(50);
    sor->setStddevMulThresh(1.0);
    sor->filter(*temp_cloud_1);

    sensor_msgs::PointCloud2 temp;
    pcl::toROSMsg(*temp_cloud_1, temp);

    // We fill the velodyne message
    msg_velo.data_nbytes = temp.data.size();
    for(int i = 0; i < temp.fields.size(); i++) {
        bot_core::pointfield_t t;
        t.name = temp.fields[i].name;
        t.count = temp.fields[i].count;
        t.offset = temp.fields[i].offset;
        t.datatype = temp.fields[i].datatype;
        msg_velo.fields.push_back(t);
    }
    msg_velo.data = temp.data;
    msg_velo.frame_id = "VELODYNE";
    msg_velo.is_bigendian = temp.is_bigendian;
    msg_velo.is_dense = temp.is_dense;
    msg_velo.nfields = temp.fields.size();
    msg_velo.point_step = temp.point_step;
    msg_velo.row_step = temp.row_step;
    msg_velo.seq = msg->header.seq;
    msg_velo.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000.0);
    msg_velo.width = temp.width;
    msg_velo.height = temp.height;

    lcmPublish_.publish("VELODYNE", &msg_velo);


    rs->setInputCloud (temp_cloud_1);
    rs->setSample(400);
    rs->setSeed(1138);
    rs->filter(*temp_cloud_2);


    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = temp_cloud_2->begin(); it
            != temp_cloud_2->end(); it++) {
        msg_velo_sub_.points.push_back({it->x, it->y, it->z});
        msg_velo_sub_.channels.push_back({});
    }
    msg_velo_sub_.n_points = msg_velo_sub_.points.size();

    lcmPublish_.publish("VELODYNE_SUBSAMPLED", &msg_velo_sub_);

}

void App::velodyne_scan_cb(const velodyne_msgs::VelodyneScanConstPtr &msg) {
    bot_core::pointcloud2_t msg_velo;
    bot_core::pointcloud_t msg_velo_sub_;

    msg_velo_sub_.channel_names = {};
    msg_velo_sub_.frame_id = "VELODYNE";
    msg_velo_sub_.seq = msg->header.seq;
    msg_velo_sub_.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000.0);
    msg_velo_sub_.n_channels = 0;


    sensor_msgs::PointCloud2 temp;

    // iterate over packet number

    velodyne_rawdata::VPointCloud packet_;
    for(int pkn = 0; pkn < msg->packets.size() - 1; pkn++) {

        data_->unpackAndFilter(msg->packets[pkn],
                               packet_,
                               msg_velo_sub_,
                               modulo_,
                               distance_,
                               pkn);
    }

    packet_.height = 1;
    packet_.width = packet_.points.size();


    std::vector<float> channels = {};
    msg_velo_sub_.channels.push_back(channels);
    pcl::toROSMsg(packet_, temp);

    msg_velo_sub_.n_points = msg_velo_sub_.points.size();


// We fill the velodyne message
    msg_velo.data_nbytes = temp.data.size();
    for(int i = 0; i < temp.fields.size(); i++) {
        bot_core::pointfield_t t;
        t.name = temp.fields[i].name;
        t.count = temp.fields[i].count;
        t.offset = temp.fields[i].offset;
        t.datatype = temp.fields[i].datatype;
        msg_velo.fields.push_back(t);
    }
    msg_velo.data = temp.data;
    msg_velo.frame_id = "VELODYNE";
    msg_velo.is_bigendian = temp.is_bigendian;
    msg_velo.is_dense = temp.is_dense;
    msg_velo.nfields = temp.fields.size();
    msg_velo.point_step = temp.point_step;
    msg_velo.row_step = temp.row_step;
    msg_velo.seq = msg->header.seq;
    msg_velo.utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000.0);
    msg_velo.width = temp.width;
    msg_velo.height = temp.height;

    lcmPublish_.publish("VELODYNE_SUBSAMPLED", &msg_velo_sub_);
    lcmPublish_.publish("VELODYNE", &msg_velo);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "ros2lcm_velodyne");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~"); // I have no idea why they do this in the original driver.

    int modulo = 2; // modulo for decimation
    float distance = 0.01; // distance in meters from the neighbor
    bool use_raw_data = true;

    if(argc >= 3) {
        modulo = atoi(argv[1]);
        distance = (float)atof(argv[2]);
    }

    if(argc >= 4) {
        use_raw_data = strcmp("false", argv[3]) == 0 ? false : true;
    }

    std::cout << "[ ROS2LCM Velodyne ] modulo: " << modulo << std::endl;
    std::cout << "[ ROS2LCM Velodyne ] max distance: " << distance << std::endl;
    std::cout << "[ ROS2LCM Velodyne ] use raw data: " << (use_raw_data ? "true" : "false") << std::endl;
    new App(nh, priv_nh, modulo, distance, use_raw_data);
    ROS_INFO("ROS2LCM Velodyne Translator Ready");

    ros::spin();
    return 0;
}
