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
            ros::Timer      vis_timer;
            bool            has_esdf;

            Eigen::Vector4d map_origin;
            Eigen::Vector4d map_size;
            Eigen::Vector4d min_boundary;
            Eigen::Vector4d max_boundary;
            Eigen::Vector4i min_idx;
            Eigen::Vector4i max_idx;
            Eigen::Vector4i voxel_num;
            double resolution, resolution_inv;
            double h_resolution, h_resolution_inv;

            vector<char> robo_buffer; 
            vector<char> robo_buffer_inv;
            vector<double> tmp_buffer1, tmp_buffer2, tmp_buffer3;
            vector<double> esdf_buffer;
            vector<double> esdf_buffer_inv;
            vector<double> esdf_buffer_all;
            vector<pair<double, Eigen::Vector3d>> cost_grad_buffer;

        public:
            MapManager();
            ~MapManager();
            inline void posToIndex(const Eigen::Vector4d& pos, Eigen::Vector4i& id);
            inline void indexToPos(const Eigen::Vector4i& id, Eigen::Vector4d& pos);
            inline int toAddress(const Eigen::Vector4i& id);
            inline int toAddress(const int& x, const int& y, const int& z, const int& h);
            inline bool isInMap(const Eigen::Vector4d& pos);
            inline bool isInMap(const Eigen::Vector4i& idx);
            void initMap(ros::NodeHandle& nh);
            void getDistWithGrad(const Eigen::Vector4d& pos, double& dist, Eigen::Vector3d& grad);
            void interPolate(double values[2][2][2][2], const Eigen::Vector4d& diff, double& value, Eigen::Vector3d& grad);
            void buildESDF(void);
            template <typename F_get_val, typename F_set_val>
            void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
            void visCallback(const ros::TimerEvent& /*event*/);
            typedef shared_ptr<MapManager> Ptr;
    };

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

    inline void MapManager::posToIndex(const Eigen::Vector4d& pos, Eigen::Vector4i& id)
    {
        for (int i = 0; i < 4; ++i) id(i) = floor((pos(i) - map_origin(i)) * resolution_inv);
    }

    inline void MapManager::indexToPos(const Eigen::Vector4i& id, Eigen::Vector4d& pos)
    {
        for (int i = 0; i < 4; ++i) pos(i) = (id(i) + 0.5) * resolution + map_origin(i);
    }
};

#endif