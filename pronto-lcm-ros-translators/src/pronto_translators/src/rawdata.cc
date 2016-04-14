/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <rawdata.h>

namespace velodyne_rawdata {
////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData() {}

/** Uppdate parameters: conversions and update */
void RawData::setParameters(double min_range,
                            double max_range,
                            double min_angle,
                            double max_angle) {

    for(int i = 0; i < SCANS_PER_BLOCK / 2; i++) {
        velo_index[i * 2] = i;
        velo_index[i * 2 + 1] = i + 16;

    }

    for(int i = 0; i < SCANS_PER_BLOCK; i++) {
        old_beams[i].distance = 0;
        old_beams[i].intensity = 0;
        old_beams[i].laser_number = 0;
        old_beams[i].rotation = 0;
        old_distances[i] = 0;
    }


    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = min_angle;
    config_.tmp_max_angle = max_angle;

    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion
    config_.min_angle = 100 * (config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle) {
        //avoid returning empty cloud if min_angle = max_angle
        config_.min_angle = 0;
        config_.max_angle = 36000;
    }

    // if for some reason min angle is greater than max, we swap them
    if(config_.min_angle > config_.max_angle){
        int tmp = config_.min_angle;
        config_.min_angle = config_.max_angle;
        config_.max_angle = tmp;
    }
    std::cout << "[ Raw Data Velodyne ] Min HW angle: " << config_.min_angle << std::endl;
    std::cout << "[ Raw Data Velodyne ] Max HW angle: " << config_.max_angle << std::endl;
}

/** Set up for on-line operation. */
int RawData::setup(ros::NodeHandle private_nh) {
    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile)) {
        ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

        // have to use something: grab unit test version as a default
        std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
        config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
    }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
        ROS_ERROR_STREAM("Unable to open calibration file: " <<
                         config_.calibrationFile);
        return -1;
    }

    ROS_INFO_STREAM("Data will be processed  ...num_lasers: " << calibration_.num_lasers);

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
        cos_rot_table_[rot_index] = cosf(rotation);
        sin_rot_table_[rot_index] = sinf(rotation);
    }
    return 0;
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt,
                     VPointCloud &pc, int ring) {
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);


    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

        for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {

            float x, y, z;
            float intensity;
            uint8_t laser_number;       ///< hardware laser number

            laser_number = j;
            velodyne_pointcloud::LaserCorrection &corrections =
                calibration_.laser_corrections[laser_number];

            /** Position Calculation */

            union two_bytes tmp;
            tmp.bytes[0] = raw->blocks[i].data[k];
            tmp.bytes[1] = raw->blocks[i].data[k + 1];
            /*condition added to avoid calculating points which are not
              in the interesting defined area (min_angle < area < max_angle)*/
            if (true) {

                float distance = tmp.uint * DISTANCE_RESOLUTION;
                distance += corrections.dist_correction;

                float cos_vert_angle = corrections.cos_vert_correction;
                float sin_vert_angle = corrections.sin_vert_correction;
                float cos_rot_correction = corrections.cos_rot_correction;
                float sin_rot_correction = corrections.sin_rot_correction;

                // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
                // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
                float cos_rot_angle =
                    cos_rot_table_[raw->blocks[i].rotation] * cos_rot_correction +
                    sin_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
                float sin_rot_angle =
                    sin_rot_table_[raw->blocks[i].rotation] * cos_rot_correction -
                    cos_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;

                float horiz_offset = corrections.horiz_offset_correction;
                float vert_offset = corrections.vert_offset_correction;

                // Compute the distance in the xy plane (w/o accounting for rotation)
                /**the new term of 'vert_offset * sin_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                float xy_distance = distance * cos_vert_angle + vert_offset * sin_vert_angle;

                // Calculate temporal X, use absolute value.
                float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
                // Calculate temporal Y, use absolute value
                float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
                if (xx < 0) xx = -xx;
                if (yy < 0) yy = -yy;

                // Get 2points calibration values,Linear interpolation to get distance
                // correction for X and Y, that means distance correction use
                // different value at different distance
                float distance_corr_x = 0;
                float distance_corr_y = 0;
                if (corrections.two_pt_correction_available) {
                    distance_corr_x =
                        (corrections.dist_correction - corrections.dist_correction_x)
                        * (xx - 2.4) / (25.04 - 2.4)
                        + corrections.dist_correction_x;
                    distance_corr_x -= corrections.dist_correction;
                    distance_corr_y =
                        (corrections.dist_correction - corrections.dist_correction_y)
                        * (yy - 1.93) / (25.04 - 1.93)
                        + corrections.dist_correction_y;
                    distance_corr_y -= corrections.dist_correction;
                }

                float distance_x = distance + distance_corr_x;
                /**the new term of 'vert_offset * sin_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                xy_distance = distance_x * cos_vert_angle + vert_offset * sin_vert_angle ;
                ///the expression wiht '-' is proved to be better than the one with '+'
                x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

                float distance_y = distance + distance_corr_y;
                xy_distance = distance_y * cos_vert_angle + vert_offset * sin_vert_angle ;
                /**the new term of 'vert_offset * sin_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

                // Using distance_y is not symmetric, but the velodyne manual
                // does this.
                /**the new term of 'vert_offset * cos_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

                /** Use standard ROS coordinate system (right-hand rule) */
                float x_coord = y;
                float y_coord = -x;
                float z_coord = z;

                /** Intensity Calculation */

                float min_intensity = corrections.min_intensity;
                float max_intensity = corrections.max_intensity;

                intensity = raw->blocks[i].data[k + 2];

                float focal_offset = 256
                                     * (1 - corrections.focal_distance / 13100)
                                     * (1 - corrections.focal_distance / 13100);
                float focal_slope = corrections.focal_slope;
                intensity += focal_slope * (abs(focal_offset - 256 *
                                                (1 - static_cast<float>(tmp.uint) / 65535) * (1 - static_cast<float>(tmp.uint) / 65535)));
                intensity = (intensity < min_intensity) ? min_intensity : intensity;
                intensity = (intensity > max_intensity) ? max_intensity : intensity;



                // convert polar coordinates to Euclidean XYZ
                VPoint point;
                point.ring = corrections.laser_ring;
                point.x = x_coord;
                point.y = y_coord;
                point.z = z_coord;
                point.intensity = (float)velo_index[j] / 31.0;

                if(ring == 32 || velo_index[j] == ring) {
                    // append this point to the cloud
                    pc.points.push_back(point);
                    ++pc.width;
                }
            }
        }
    }
}

void RawData::unpackAndFilter(const velodyne_msgs::VelodynePacket &pkt,
                              VPointCloud &pc,
                              bot_core::pointcloud_t &pc_t,
                              int modulo,
                              float distance,
                              int pktnum) {


    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    VPoint point;
    std::vector<float> point_lcm = {0.0, 0.0, 0.0};
    std::vector<float> channel_lcm = {0.0, 0.0};


    // iterate over blocks
    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
        // iterate over laser per block
        for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {

            // Fill the beam
            Beam b;
            b.rotation = raw->blocks[i].rotation;
            b.laser_number = j; ///< hardware laser number
            union two_bytes tmp;
            tmp.bytes[0] = raw->blocks[i].data[k];
            tmp.bytes[1] = raw->blocks[i].data[k + 1];
            b.distance = tmp.uint;


            // IF the rotation is within the range we chose
            // AND
            //    the distance is withing the range we chose
            // AND
            //    the distance difference with the previous value is less than
            //    the maximum difference we chose
            // AND
            //    the distance is not zero
            // AND
            //    the previous distance is zero
            //
            // we can proceed :)

            if(raw->blocks[i].rotation >= config_.min_angle
                    && raw->blocks[i].rotation <= config_.max_angle
                    && b.distance < config_.max_range / DISTANCE_RESOLUTION
                    && b.distance > config_.min_range / DISTANCE_RESOLUTION
                    && std::abs(b - old_beams[j]) < (int)std::round(distance / DISTANCE_RESOLUTION)
                    && b.distance != 0
                    && old_beams[j].distance != 0) {


                getPointFromBeam(b, point);


                // We fill the point cloud with decimated points
                if((pktnum * BLOCKS_PER_PACKET + i) % modulo == 0)   {
                    point_lcm = {point.x, point.y, point.z};
                    channel_lcm = {};
                    pc_t.points.push_back(point_lcm);
                    pc_t.channels.push_back(channel_lcm);
                }

                // append this point to the cloud
                pc.points.push_back(point);

                //++pc.width;
            }
            old_beams[j] = b;
        }
    }
}


void RawData::getPointFromBeam(const Beam &b,
                               VPoint &point) {

    velodyne_pointcloud::LaserCorrection corrections =
        calibration_.laser_corrections[b.laser_number];
    float x, y, z;

    float distance = b.distance * DISTANCE_RESOLUTION;
    distance += corrections.dist_correction;

    float cos_vert_angle = corrections.cos_vert_correction;
    float sin_vert_angle = corrections.sin_vert_correction;
    float cos_rot_correction = corrections.cos_rot_correction;
    float sin_rot_correction = corrections.sin_rot_correction;

    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    float cos_rot_angle =
        cos_rot_table_[b.rotation] * cos_rot_correction +
        sin_rot_table_[b.rotation] * sin_rot_correction;
    float sin_rot_angle =
        sin_rot_table_[b.rotation] * cos_rot_correction -
        cos_rot_table_[b.rotation] * sin_rot_correction;

    float horiz_offset = corrections.horiz_offset_correction;
    float vert_offset = corrections.vert_offset_correction;

    // Compute the distance in the xy plane (w/o accounting for rotation)
    /**the new term of 'vert_offset * sin_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    float xy_distance = distance * cos_vert_angle + vert_offset * sin_vert_angle;

    // Calculate temporal X, use absolute value.
    float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    // Calculate temporal Y, use absolute value
    float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    if (xx < 0) xx = -xx;
    if (yy < 0) yy = -yy;

    // Get 2points calibration values,Linear interpolation to get distance
    // correction for X and Y, that means distance correction use
    // different value at different distance
    float distance_corr_x = 0;
    float distance_corr_y = 0;
    if (corrections.two_pt_correction_available) {
        distance_corr_x =
            (corrections.dist_correction - corrections.dist_correction_x)
            * (xx - 2.4) / (25.04 - 2.4)
            + corrections.dist_correction_x;
        distance_corr_x -= corrections.dist_correction;
        distance_corr_y =
            (corrections.dist_correction - corrections.dist_correction_y)
            * (yy - 1.93) / (25.04 - 1.93)
            + corrections.dist_correction_y;
        distance_corr_y -= corrections.dist_correction;
    }

    float distance_x = distance + distance_corr_x;
    /**the new term of 'vert_offset * sin_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    xy_distance = distance_x * cos_vert_angle + vert_offset * sin_vert_angle ;
    ///the expression wiht '-' is proved to be better than the one with '+'
    x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

    float distance_y = distance + distance_corr_y;
    xy_distance = distance_y * cos_vert_angle + vert_offset * sin_vert_angle ;
    /**the new term of 'vert_offset * sin_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

    // Using distance_y is not symmetric, but the velodyne manual
    // does this.
    /**the new term of 'vert_offset * cos_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

    /** Use standard ROS coordinate system (right-hand rule) */


    point.x = y;
    point.y = -x;
    point.z = z;
//    point[0] = y;
//    point[1] = -x;
//    point[2] = z;

//    float intensity = b.intensity;


//    float min_intensity = corrections.min_intensity;
//    float max_intensity = corrections.max_intensity;



//    float focal_offset = 256
//                         * (1 - corrections.focal_distance / 13100)
//                         * (1 - corrections.focal_distance / 13100);
//    float focal_slope = corrections.focal_slope;
//    intensity += focal_slope * (abs(focal_offset - 256 *
//                                    (1 - static_cast<float>(b.distance) / 65535) * (1 - static_cast<float>(b.distance) / 65535)));
//    intensity = (intensity < min_intensity) ? min_intensity : intensity;
//    intensity = (intensity > max_intensity) ? max_intensity : intensity;

//    vp.x = x_c;
//    vp.y = y_c;
//    vp.z = z_c;
//    vp.intensity = (uint8_t) intensity;
//    vp.ring = corrections.laser_ring;
}

} // namespace velodyne_rawdata
