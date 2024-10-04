/**
 * This file is part of slict.
 *
 * Copyright (C) 2020 Thien-Minh Nguyen <thienminh.npn at ieee dot org>,
 * Division of RPL, KTH Royal Institute of Technology
 *
 * For more information please see <https://britsknguyen.github.io>.
 * or <https://github.com/brytsknguyen/slict>.
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * slict is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * slict is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with slict.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// Created by Thien-Minh Nguyen on 01/08/22.
//

#include "utility.h"

#include <livox_ros_driver2/CustomMsg.h>

// const int queueLength = 2000;

using namespace std;
using namespace Eigen;
using namespace pcl;

class LivoxPoints2ToOuster
{
private:
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    ros::Subscriber livoxCloudSub;
    ros::Publisher ousterCloudPub;

    double intensityConvCoef = -1;

    int NUM_CORE;

    // bool remove_human_body = true;

public:
    // Destructor
    ~LivoxPoints2ToOuster() {}

    LivoxPoints2ToOuster(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        NUM_CORE = omp_get_max_threads();

        // Coefficient to convert the intensity from livox to ouster
        nh_ptr->param("intensityConvCoef", intensityConvCoef, 1.0);

        livoxCloudSub = nh_ptr->subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 50, &LivoxPoints2ToOuster::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        ousterCloudPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/livox/lidar_ouster", 50);
    }

    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
    {
        int cloudsize = msgIn->width * msgIn->height;

        CloudOuster laserCloudOuster;
        laserCloudOuster.points.reserve(cloudsize);
        laserCloudOuster.is_dense = true;

        const int intensity_offset = 12;    // float
        const int line_offset = 17;         // uint8
        const int time_offset = 18;         // double
        
        const double t0 = *reinterpret_cast<const double*>(msgIn->data.data() + time_offset);

        for (size_t i = 0; i < cloudsize; i++)
        {
            PointOuster dst;
            const unsigned char* pt = msgIn->data.data() + i * msgIn->point_step;
            dst.x = reinterpret_cast<const float*>(pt)[0];
            dst.y = reinterpret_cast<const float*>(pt)[1];
            dst.z = reinterpret_cast<const float*>(pt)[2];
            dst.intensity = *reinterpret_cast<const float*>(pt + intensity_offset) * intensityConvCoef;
            dst.ring = *reinterpret_cast<const uint8_t*>(pt + line_offset);
            dst.t = (*reinterpret_cast<const double*>(pt + time_offset) - t0);
            dst.range = sqrt(dst.x * dst.x + dst.y * dst.y + dst.z * dst.z)*1000;

            if (dst.y < 0) {
                continue;
            }
            laserCloudOuster.points.push_back(dst);
        }

        Util::publishCloud(ousterCloudPub, laserCloudOuster, msgIn->header.stamp, msgIn->header.frame_id);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_to_ouster");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO(KGRN "----> Velodyne to Ouster started" RESET);

    LivoxPoints2ToOuster C2P(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}