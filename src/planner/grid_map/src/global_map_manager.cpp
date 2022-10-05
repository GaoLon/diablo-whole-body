#include "grid_map/global_map_manager.h"
#include <visualization_msgs/Marker.h>

namespace ugv_planner
{
    MapManager::MapManager(): \
        map_size(Eigen::Vector4d(0.8, 0.9, 0.9, 0.16)),\
        has_esdf(false)
    { 
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        min_boundary(2) = 0.0;
        max_boundary(2) = map_size(2);
        min_boundary(3) = 0.0;
        max_boundary(3) = map_size(3);
        map_origin = min_boundary;
    }

    void MapManager::initMap(ros::NodeHandle& node_) 
    {
        // init param
        nh = node_;
        node_.param("map/resolution", resolution, -1.0);       
        node_.param("map/h_resolution", h_resolution, -1.0);
        vis_timer = node_.createTimer(ros::Duration(1.0), &MapManager::visCallback, this);
        esdf_pub  = node_.advertise<sensor_msgs::PointCloud2>("robo_esdf", 10);
        field_pub = node_.advertise<sensor_msgs::PointCloud2>("robo_field", 10);

        resolution_inv = 1.0 / resolution;
        h_resolution_inv = 1.0 / h_resolution;

        for (size_t i=0; i<3; i++)
            voxel_num(i) = ceil(map_size(i) / resolution);
        voxel_num(3) = ceil(map_size(3) / h_resolution);
        min_idx = Eigen::Vector4i::Zero();
        max_idx = voxel_num - Eigen::Vector4i::Ones();

        // init buffer
        int buffer_size = voxel_num(0) * voxel_num(1) * voxel_num(2) * voxel_num(3);
        robo_buffer      = vector<char>(buffer_size, 0);
        robo_buffer_inv  = vector<char>(buffer_size, 0);
        esdf_buffer      = vector<double>(buffer_size, 10000);
        esdf_buffer_inv  = vector<double>(buffer_size, 10000);
        esdf_buffer_all  = vector<double>(buffer_size, 10000);
        tmp_buffer1      = vector<double>(buffer_size, 10000);
        tmp_buffer2      = vector<double>(buffer_size, 10000);
        tmp_buffer3      = vector<double>(buffer_size, 10000);
        cost_grad_buffer = vector<pair<double, Eigen::Vector3d>>(buffer_size, pair<double, Eigen::Vector3d>(0.0, Eigen::Vector3d(0.0, 0.0, 0.0)));

        // reset buffer
        double half_res = resolution * 0.9;
        double half_hres = h_resolution * 0.9;
        for (double x=min_boundary(0); x<max_boundary(0); x+=half_res)
            for (double y=min_boundary(1); y<max_boundary(1); y+=half_res)
                for (double z=min_boundary(2); z<max_boundary(2); z+=half_res)
                    for (double h=min_boundary(3); h<max_boundary(3); h+=half_res)
                    {
                        if (isInMap(Eigen::Vector4d(x, y, z, h)))
                        {
                            if (x<=0.1 && x>=-0.1 && y<=0.25 && y>=0.15 && z<=0.3 && z>=0.0)
                            {
                                robo_buffer[toAddress(x, y, z, h)] = 1;
                            }
                            else if (x<=0.1 && x>=-0.1 && y<=-0.15 && y>=-0.25 && z<=0.3 && z>=0.0)
                            {
                                robo_buffer[toAddress(x, y, z, h)] = 1;
                            }
                            else if (x<=0.2 && x>=-0.2 && y<=0.15 && y>=-0.15 && z<=0.48+h && z>=0.08+h)
                            {
                                robo_buffer[toAddress(x, y, z, h)] = 1;
                            }
                        }
                    } 
        
        // build ESDF
        buildESDF();
    }

    void MapManager::visCallback(const ros::TimerEvent& /*event*/)
    {
        if (!has_esdf)
            return;

        // robo esdf vis
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointXYZI pt;
        for (int x=min_idx[0]; x<=max_idx[0]; x++)
            for (int y=min_idx[0]; y<=max_idx[0]; y++)
                for (int z=min_idx[0]; z<=max_idx[0]; z++)
                {
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.intensity = esdf_buffer_all[toAddress(x, y, z, 0.08)];
                    cloud.push_back(pt);
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
        double dist_field;
        Eigen::Vector3d esdf_field;
        for (double x=min_boundary(0); x<max_boundary(0); x+=0.01)
            for (double y=min_boundary(1); y<max_boundary(1); y+=0.01)
                for (double z=min_boundary(2); z<max_boundary(2); z+=0.01)
                {
                    getDistWithGrad(Eigen::Vector3d(x, y, z, 0.08), dist_field, esdf_field);
                    pt_field.x = x - 10.0;
                    pt_field.y = y - 10.0;
                    pt_field.z = z + 1.0;
                    pt_field.intensity = dist_field;
                    cloud_field.push_back(pt_field);
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

    // TODO: add height, change field
    void MapManager::buildESDF()
    {
       /* ========== compute positive DT ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF([&](int z) {
                    return robo_buffer[toAddress(x, y, z)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int z, double val) { tmp_buffer1[toAddress(x, y, z)] = val; }, min_idx[2],
                max_idx[2], 2);
            }
        }

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
            fillESDF([&](int y) { return tmp_buffer1[toAddress(x, y, z)]; },
                    [&](int y, double val) { tmp_buffer2[toAddress(x, y, z)] = val; }, min_idx[1],
                    max_idx[1], 1);
            }
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
            fillESDF([&](int x) { return tmp_buffer2[toAddress(x, y, z)]; },
                    [&](int x, double val) {
                        esdf_buffer[toAddress(x, y, z)] = resolution * std::sqrt(val);//开根号求距离
                    },
                    min_idx[0], max_idx[0], 0);
            }
        }

        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            for (int z = min_idx(2); z <= max_idx(2); ++z) {

                int idx = toAddress(x, y, z);
                if (robo_buffer[idx] == 0) {
                robo_buffer_inv[idx] = 1;

                } else if (robo_buffer[idx] == 1) {
                robo_buffer_inv[idx] = 0;
                } else {
                ROS_ERROR("what?");
                }
            }

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF([&](int z) {//x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
                    return robo_buffer_inv[toAddress(x, y, z)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int z, double val) { tmp_buffer1[toAddress(x, y, z)] = val; }, min_idx[2],
                max_idx[2], 2);
            }
        }

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
            fillESDF([&](int y) { return tmp_buffer1[toAddress(x, y, z)]; },
                    [&](int y, double val) { tmp_buffer2[toAddress(x, y, z)] = val; }, min_idx[1],
                    max_idx[1], 1);
            }
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
            fillESDF([&](int x) { return tmp_buffer2[toAddress(x, y, z)]; },
                    [&](int x, double val) {
                        esdf_buffer_inv[toAddress(x, y, z)] = resolution * std::sqrt(val);
                    },
                    min_idx[0], max_idx[0], 0);
            }
        }
        
        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            for (int z = min_idx(2); z <= max_idx(2); ++z) {

                int idx = toAddress(x, y, z);

                esdf_buffer_all[idx] = 0.0;

                if (esdf_buffer_inv[idx] > 0.0)
                    esdf_buffer_all[idx] += (-esdf_buffer_inv[idx] + resolution);//加reso是为了让正负梯度切换更加平滑 
            }
        
        has_esdf = true;
        return;
    }

    // TODO: to four
    void MapManager::getDistWithGrad(const Eigen::Vector4d& pos, double& dist, Eigen::Vector3d& grad)
    {
        Eigen::Vector4i pos_id = Eigen::Vector3i(int(floor((pos(0)+1.1)*mp_.resolution_inv_)), int(floor((pos(1)+0.8)*mp_.resolution_inv_)), int(floor((pos(2)+0.5)*mp_.resolution_inv_)));
        Eigen::Vector4d pts[2][2][2];
        Eigen::Vector4i pos_dist_id;
        Eigen::Vector4d diff;
        diff = Eigen::Vector4d((pos(0)-(pos_id(0)*mp_.resolution_-1.1))*mp_.resolution_inv_, (pos(1)-(pos_id(1)*mp_.resolution_-0.8))*mp_.resolution_inv_, (pos(2)-(pos_id(2)*mp_.resolution_-0.5))*mp_.resolution_inv_);
        int pos_idx;
        double dists[2][2][2][2];
        // cout<<"======================="<<endl;
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++) 
            for (int z = 0; z < 2; z++){
                pos_dist_id = pos_id + Eigen::Vector3i(x, y, z);
                pos_idx = pos_dist_id(0)*17*11+pos_dist_id(1)*11+pos_dist_id(2);
                dists[x][y][z] =  md_.fullshape3d_distance_buffer_all_[pos_idx];

            }
        interPolate(dists, diff, dist, grad); 
    }

    // TODO: to four
    void MapManager::interPolate(double values[2][2][2][2], const Eigen::Vector4d& diff, double& value, Eigen::Vector3d& grad)
    {
        // trilinear interpolation
        double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
        double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
        double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
        double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
        double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
        double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

        value = (1 - diff(2)) * v0 + diff(2) * v1;

        grad[2] = (v1 - v0) * resolution_inv;
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= resolution_inv;
    }
}
