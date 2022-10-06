#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string.h>

#define inf 1>>20
using namespace std;

namespace ugv_planner
{
    class MapManager
    {
        private:
            ros::NodeHandle nh;
            ros::Publisher  esdf_pub;
            ros::Publisher  field_pub;
            ros::Subscriber cloud_sub;
            ros::Timer      vis_timer;
            bool            has_esdf;
            bool            has_cloud;
            double          debug_h;
            int             debug_h_idx;

            Eigen::Vector4d map_origin;
            Eigen::Vector4d map_size;
            Eigen::Vector4d min_boundary;
            Eigen::Vector4d max_boundary;
            Eigen::Vector4i min_idx;
            Eigen::Vector4i max_idx;
            Eigen::Vector4i voxel_num;
            double resolution, resolution_inv;
            double h_resolution, h_resolution_inv;
            double h2_r2;

            vector<char> robo_buffer;
            vector<double> tmp_buffer1, tmp_buffer2;
            vector<double> esdf_buffer_inv;
            vector<double> esdf_buffer_all;
            pcl::PointCloud<pcl::PointXYZ> env_cloud;

        public:
            MapManager();
            ~MapManager() {}
            inline void posToIndex(const Eigen::Vector4d& pos, Eigen::Vector4i& id);
            inline void indexToPos(const Eigen::Vector4i& id, Eigen::Vector4d& pos);
            inline int toAddress(const Eigen::Vector4i& id);
            inline int toAddress(const int& x, const int& y, const int& z, const int& h);
            inline bool isInMap(const Eigen::Vector4d& pos);
            inline bool isInMap(const Eigen::Vector4i& idx);
            inline int isOccupancy(Eigen::Vector4d pos);
            inline int isOccupancy(Eigen::Vector4i id);
            inline void boundIndex(Eigen::Vector4i& id);
            inline pair<double, double> getResolution(void);
            inline void getDistWithGrad(const Eigen::Vector4d& pos, double& dist, Eigen::Vector3d& grad);
            void initMap(ros::NodeHandle& nh);
            void interPolate(double values[2][2][2][2], const Eigen::Vector4d& diff, double& value, Eigen::Vector3d& grad);
            void buildESDF(void);
            template <typename F_get_val, typename F_set_val>
            void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
            void visCallback(const ros::TimerEvent& /*event*/);
            void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
            vector<Eigen::Vector3d> frontEndSearch(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos);
            typedef shared_ptr<MapManager> Ptr;
    };

    inline void MapManager::posToIndex(const Eigen::Vector4d& pos, Eigen::Vector4i& id)
    {
        for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - map_origin(i)) * resolution_inv);
        id(3) = floor((pos(3) - map_origin(3)) * h_resolution_inv);
    }

    inline void MapManager::indexToPos(const Eigen::Vector4i& id, Eigen::Vector4d& pos)
    {
        for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * resolution + map_origin(i);
        pos(3) = (id(3) + 0.5) * h_resolution + map_origin(3);
    }

    inline int MapManager::toAddress(const Eigen::Vector4i& id) 
    {
        return id(0) * voxel_num(1)*voxel_num(2)*voxel_num(3) + id(1) * voxel_num(2)*voxel_num(3) + id(2) * voxel_num(3) + id(3);
    }

    inline int MapManager::toAddress(const int& x, const int& y, const int& z, const int& h) 
    {
        return x * voxel_num(1)*voxel_num(2)*voxel_num(3) + y * voxel_num(2)*voxel_num(3) + z * voxel_num(3) + h;
    }
    
    inline bool MapManager::isInMap(const Eigen::Vector4d& pos) 
    {
        if (pos(0) < min_boundary(0) + 1e-4 || \
            pos(1) < min_boundary(1) + 1e-4 || \
            pos(2) < min_boundary(2) + 1e-4 || \
            pos(3) < min_boundary(3) + 1e-4     ) 
        {
            return false;
        }

        if (pos(0) > max_boundary(0) + 1e-4 || \
            pos(1) > max_boundary(1) + 1e-4 || \
            pos(2) > max_boundary(2) + 1e-4 || \
            pos(3) > max_boundary(3) + 1e-4     ) 
        {
            return false;
        }

        return true;
    }

    inline bool MapManager::isInMap(const Eigen::Vector4i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0 || idx(3) < 0)
        {
            return false;
        }

        if (idx(0) > voxel_num(0) - 1 || \
            idx(1) > voxel_num(1) - 1 || \
            idx(2) > voxel_num(2) - 1 || \
            idx(3) > voxel_num(3) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline int MapManager::isOccupancy(Eigen::Vector4d pos)
    {
        Eigen::Vector4i id;

        posToIndex(pos, id);
        
        return isOccupancy(id);
    }

    inline int MapManager::isOccupancy(Eigen::Vector4i id)
    {
        if (!isInMap(id))
            return -1;

        return int(robo_buffer[toAddress(id)]);
    }

    inline void MapManager::boundIndex(Eigen::Vector4i& id)
    {
        Eigen::Vector4i id1;
        id1(0) = max(min(id(0), max_idx(0)), min_idx(0));
        id1(1) = max(min(id(1), max_idx(1)), min_idx(1));
        id1(2) = max(min(id(2), max_idx(2)), min_idx(2));
        id1(3) = max(min(id(3), max_idx(3)), min_idx(3));
        id = id1;
    }

    inline pair<double, double> MapManager::getResolution(void)
    {
        return make_pair(resolution, h_resolution);
    }

    inline void MapManager::getDistWithGrad(const Eigen::Vector4d& pos, double& dist, Eigen::Vector3d& grad)
    {
        if (!isInMap(pos))
        {
            grad.setZero();
            dist = 0.0;
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector4d pos_m = pos;
        for (size_t i=0; i<2; i++)
        {
            pos_m(i) -= 0.5 * resolution;
        }
        pos_m(3) -= 0.5 * h_resolution;

        Eigen::Vector4i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector4d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector4d diff = pos - idx_pos;
        for (size_t i=0; i<2; i++)
        {
            diff(i) *= resolution_inv;
        }
        diff(3) *= h_resolution_inv;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int h = 0; h < 2; h++)
                {
                    Eigen::Vector4i current_idx = idx + Eigen::Vector4i(x, y, 0, h);
                    boundIndex(current_idx);
                    values[x][y][h] = esdf_buffer_all[toAddress(current_idx)];
                }

        double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
        double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
        double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
        double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
        double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
        double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
        dist = (1 - diff[3]) * v0 + diff[3] * v1;

        grad[2] = (v1 - v0) * h_resolution_inv;
        grad[1] = ((1 - diff[3]) * (v10 - v00) + diff[3] * (v11 - v01)) * resolution_inv;
        grad[0] = (1 - diff[3]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[3]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[3] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[3] * diff[1] * (values[1][1][1] - values[0][1][1]);

        grad[0] *= resolution_inv;

        return;
    }
    
};

#endif