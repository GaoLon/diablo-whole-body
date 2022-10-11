#include "robo_map/robo_map_manager.h"
#include <visualization_msgs/Marker.h>

namespace ugv_planner
{
    MapManager::MapManager(): \
        map_size(Eigen::Vector4d(0.8, 0.9, 0.8, 1.08)),\
        has_esdf(false), env_cloud(new pcl::PointCloud<pcl::PointXYZ>),\
        world_size(Eigen::Vector3d(18.0, 8.0, 2.0)), world_resolution(0.02)
    { 
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        min_boundary(2) = -0.1;
        max_boundary(2) = min_boundary(2) + map_size(2);
        min_boundary(3) = -0.5;
        max_boundary(3) = min_boundary(3) + map_size(3);
        map_origin = min_boundary;
        
        world_resolution_inv = 1.0 / world_resolution;
        world_origin = world_min_boundary = -world_size / 2.0;
        world_max_boundary = world_size / 2.0;
        for (size_t i=0; i<3; i++)
            world_voxel_num(i) = ceil(world_size(i) / world_resolution);
        world_buffer = vector<char>(world_voxel_num(0) * world_voxel_num(1) * world_voxel_num(2), 0);
        plane_buffer = vector<char>(world_voxel_num(0) * world_voxel_num(1), 0);
    }

    void MapManager::initMap(ros::NodeHandle& node_) 
    {
        // init param
        nh = node_;
        node_.param("robomap/resolution", resolution, -1.0);       
        node_.param("robomap/h_resolution", h_resolution, -1.0);
        node_.param("robomap/leaf_size", leaf_size, -1.0);
        node_.param("robomap/debug_h", debug_h, 100.0);
        vis_timer = node_.createTimer(ros::Duration(1.0), &MapManager::visCallback, this);
        esdf_pub  = node_.advertise<sensor_msgs::PointCloud2>("robo_esdf", 10);
        field_pub = node_.advertise<sensor_msgs::PointCloud2>("robo_field", 10);
        cloud_sub = node_.subscribe<sensor_msgs::PointCloud2>("/global_map", 10, &MapManager::cloudCallback, this);

        resolution_inv = 1.0 / resolution;
        h_resolution_inv = 1.0 / h_resolution;
        h2_r2 = pow(h_resolution/resolution, 2);
        debug_h_idx = floor((debug_h - map_origin(3)) * h_resolution_inv);

        for (size_t i=0; i<3; i++)
            voxel_num(i) = ceil(map_size(i) / resolution);
        voxel_num(3) = ceil(map_size(3) / h_resolution);
        min_idx = Eigen::Vector4i::Zero();
        max_idx = voxel_num - Eigen::Vector4i::Ones();

        // init buffer
        int buffer_size  = voxel_num(0) * voxel_num(1) * voxel_num(2) * voxel_num(3);
        robo_buffer      = vector<char>(buffer_size, 0);
        esdf_buffer_inv  = vector<double>(buffer_size, 10000);
        esdf_buffer_all  = vector<double>(buffer_size, 10000);
        tmp_buffer1      = vector<double>(buffer_size, 10000);
        tmp_buffer2      = vector<double>(buffer_size, 10000);

        // reset buffer
        double half_res = resolution * 0.4;
        double half_hres = h_resolution * 0.4;
        for (double x=min_boundary(0); x<=max_boundary(0); x+=half_res)
            for (double y=min_boundary(1); y<=max_boundary(1); y+=half_res)
                for (double z=min_boundary(2); z<=max_boundary(2); z+=half_res)
                    for (double h=min_boundary(3); h<=max_boundary(3); h+=half_hres)
                    {
                        Eigen::Vector4d pos(x, y, z, h);
                        if (isInMap(pos))
                        {
                            Eigen::Vector4i idx;
                            posToIndex(pos, idx);
                            if (x<=0.1 && x>=-0.1 && y<=0.25 && y>=0.15 && z<=0.3 && z>=0.0)
                            {
                                robo_buffer[toAddress(idx)] = 1;
                            }
                            else if (x<=0.1 && x>=-0.1 && y<=-0.15 && y>=-0.25 && z<=0.3 && z>=0.0)
                            {
                                robo_buffer[toAddress(idx)] = 1;
                            }
                            else if (x<=0.2 && x>=-0.2 && y<=0.15 && y>=-0.15 && z<=0.48+h && z>=0.08+h)
                            {
                                robo_buffer[toAddress(idx)] = 1;
                            }
                        }
                    } 
        
        // build ESDF
        buildESDF();
    }

    void MapManager::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        if (!has_cloud)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> DwzFilter;
        
            pcl::fromROSMsg(*cloud, *cloudMap);
            DwzFilter.setLeafSize(leaf_size, leaf_size, leaf_size);
            DwzFilter.setInputCloud(cloudMap);
            DwzFilter.filter(*env_cloud);
            cloudMap->clear();

            // pcl::fromROSMsg(*cloud, *env_cloud);

            kdtree.setInputCloud(env_cloud);
            for (size_t i=0; i<env_cloud->points.size(); i++)
            {
                Eigen::Vector3d p(env_cloud->points[i].x, env_cloud->points[i].y, env_cloud->points[i].z);
                Eigen::Vector3i idx;

                posToIndexWorld(p, idx);

                if (!isInMapWorld(idx)) continue;

                world_buffer[toAddressWorld(idx)] = 1;
            }

            ROS_INFO("get point clouds.");
            has_cloud = true;
        }

        return;
    }

    void MapManager::visCallback(const ros::TimerEvent& /*event*/)
    {
        if (!has_esdf || debug_h < 0.0 || debug_h > 0.16)
            return;

        // robo esdf vis
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointXYZI pt;
        for (int x=min_idx[0]; x<=max_idx[0]; x++)
            for (int y=min_idx[1]; y<=max_idx[1]; y++)
                for (int z=min_idx[2]; z<=max_idx[2]; z++)
                {
                    if (robo_buffer[toAddress(x, y, z, debug_h_idx)] == 1)
                    {
                        pt.x = x;
                        pt.y = y;
                        pt.z = z;
                        pt.intensity = esdf_buffer_all[toAddress(x, y, z, debug_h_idx)];
                        cloud.push_back(pt);
                    }
                }
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "world";
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        esdf_pub.publish(cloud_msg);

        // robo field vis
        pcl::PointCloud<pcl::PointXYZI> cloud_field;
        pcl::PointXYZI pt_field;
        double dist;
        Eigen::Vector3d esdf_field;
        for (double x=min_boundary(0); x<=max_boundary(0); x+=0.01)
            for (double y=min_boundary(1); y<=max_boundary(1); y+=0.01)
                for (double z=min_boundary(2); z<=max_boundary(2); z+=0.01)
                {
                    Eigen::Vector4d pos(x, y, z, debug_h);
                    Eigen::Vector4i idx;
                    if (isInMap(pos))
                    {
                        getDistWithGrad(pos, dist, esdf_field);
                        posToIndex(pos, idx);
                        if (robo_buffer[toAddress(idx)] == 1)
                        {
                            pt_field.x = x;
                            pt_field.y = y;
                            pt_field.z = z + 1.0;
                            pt_field.intensity = dist;
                            cloud_field.push_back(pt_field);
                        }
                    }
                }
        cloud_field.width = cloud_field.points.size();
        cloud_field.height = 1;
        cloud_field.is_dense = true;
        cloud_field.header.frame_id = "world";
        sensor_msgs::PointCloud2 cloud_msg_field;
        pcl::toROSMsg(cloud_field, cloud_msg_field);
        field_pub.publish(cloud_msg_field);
    }

    template <typename F_get_val, typename F_set_val>
    void MapManager::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
    {
        int v[voxel_num(dim)];
        double z[voxel_num(dim) + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++)
        {
            k++;
            double s;

            do {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++)
        {
            while (z[k + 1] < q) k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    void MapManager::buildESDF()
    {
        for (int z = min_idx[2]; z <= max_idx[2]; z++)
            for (int y = min_idx[1]; y <= max_idx[1]; y++)
                for (int x = min_idx[0]; x <= max_idx[0]; x++)
                {
                    fillESDF([&](int h) {
                            return robo_buffer[toAddress(x, y, z, h)] == 0 ?
                                0 :
                                std::numeric_limits<double>::max();
                        },
                        [&](int h, double val) { tmp_buffer1[toAddress(x, y, z, h)] = val*h2_r2; }, min_idx[3],
                        max_idx[3], 3);
                }      

        for (int z = min_idx[2]; z <= max_idx[2]; z++)
            for (int y = min_idx[1]; y <= max_idx[1]; y++)
                for (int h = min_idx[3]; h <= max_idx[3]; h++)
                {
                    fillESDF([&](int x) { return tmp_buffer1[toAddress(x, y, z, h)]; },
                        [&](int x, double val) { tmp_buffer2[toAddress(x, y, z, h)] = val; }, min_idx[0],
                        max_idx[0], 0);
                }
        
        for (int z = min_idx[2]; z <= max_idx[2]; z++)
            for (int h = min_idx[3]; h <= max_idx[3]; h++)
                for (int x = min_idx[0]; x <= max_idx[0]; x++)
                {
                    fillESDF([&](int y) { return tmp_buffer2[toAddress(x, y, z, h)]; },
                        [&](int y, double val) {
                            esdf_buffer_inv[toAddress(x, y, z, h)] = resolution * std::sqrt(val);
                        },
                        min_idx[1], max_idx[1], 1);
                }
        
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
                for (int z = min_idx(2); z <= max_idx(2); ++z)
                    for (int h = min_idx(3); h <= max_idx(3); ++h)
                    {
                        int idx = toAddress(x, y, z, h);
                        esdf_buffer_all[idx] = 0.0;
                        if (esdf_buffer_inv[idx] > 0.0)
                        {
                            esdf_buffer_all[idx] += (-esdf_buffer_inv[idx]);
                            // esdf_buffer_all[idx] += (-esdf_buffer_inv[idx] + resolution);
                        }
                    }

        has_esdf = true;
        ROS_INFO("robo esdf build down.");
        return;
    }
    
}
