#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string.h>

// #define AABB
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
            ros::Publisher  world_pub;
            ros::Subscriber cloud_sub;
            ros::Timer      vis_timer;
            bool            has_esdf;
            bool            has_cloud;
            double          debug_h;
            int             debug_h_idx;

            // robo map param and data
            Eigen::Vector4d map_origin;
            Eigen::Vector4d map_size;
            Eigen::Vector4d min_boundary;
            Eigen::Vector4d max_boundary;
            Eigen::Vector4i min_idx;
            Eigen::Vector4i max_idx;
            Eigen::Vector4i voxel_num;
            double resolution, resolution_inv;
            double h_resolution, h_resolution_inv;
            double leaf_size;
            double h2_r2;

            vector<char> robo_buffer;
            vector<double> tmp_buffer1, tmp_buffer2;
            vector<double> esdf_buffer_inv;
            vector<double> esdf_buffer_all;

        public:
            MapManager();
            ~MapManager() {}
            // robo map function
            inline void posToIndex(const Eigen::Vector4d& pos, Eigen::Vector4i& id);
            inline void indexToPos(const Eigen::Vector4i& id, Eigen::Vector4d& pos);
            inline int toAddress(const Eigen::Vector4i& id);
            inline int toAddress(const int& x, const int& y, const int& z, const int& h);
            inline bool isInMap(const Eigen::Vector4d& pos);
            inline bool isInMap(const Eigen::Vector4i& idx);
            inline int isOccupancy(const Eigen::Vector4d& pos);
            inline int isOccupancy(const Eigen::Vector4i& id);
            inline bool isStateFree(const Eigen::Vector4d& state);
            inline bool isHStateFree(const Eigen::Vector3d& state, double& h);
            inline void boundIndex(Eigen::Vector4i& id);
            inline pair<double, double> getResolution(void);
            inline void getDistWithGrad(const Eigen::Vector4d& pos, double& dist, Eigen::Vector3d& grad);
            void initMap(ros::NodeHandle& nh);
            void buildESDF(void);
            template <typename F_get_val, typename F_set_val>
            void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
            void visCallback(const ros::TimerEvent& /*event*/);
            void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

            // world and 2.5D map param and data
            Eigen::Vector3d world_origin;
            Eigen::Vector3d world_size;
            Eigen::Vector3d world_min_boundary;
            Eigen::Vector3d world_max_boundary;
            Eigen::Vector3i world_voxel_num;
            double world_resolution, world_resolution_inv;
            vector<char> world_buffer;
            vector<char> plane_buffer;
            pcl::PointCloud<pcl::PointXYZ>::Ptr env_cloud;
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            
            // world and 2.5D map function
            inline void posToIndexWorld(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
            inline Eigen::Vector2i posToIndexPlane(const Eigen::Vector2d& pos);
            inline void indexToPosWorld(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
            inline int toAddressWorld(const Eigen::Vector3i& id);
            inline int toAddressWorld(const int& x, const int& y, const int& z);
            inline bool isInMapWorld(const Eigen::Vector3d& pos);
            inline bool isInMapWorld(const Eigen::Vector3i& idx);
            inline bool isInMapPlane(const Eigen::Vector3d& pos);
            inline bool isInMapPlane(const Eigen::Vector3i& idx);
            inline int isOccupancyWorld(const Eigen::Vector3d& pos);
            inline int isOccupancyWorld(const Eigen::Vector3i& id);

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

    inline int MapManager::isOccupancy(const Eigen::Vector4d& pos)
    {
        Eigen::Vector4i id;

        posToIndex(pos, id);
        
        return isOccupancy(id);
    }

    inline int MapManager::isOccupancy(const Eigen::Vector4i& id)
    {
        if (!isInMap(id))
            return -1;

        return int(robo_buffer[toAddress(id)]);
    }

    inline bool MapManager::isStateFree(const Eigen::Vector4d& state)
    {
#ifdef AABB
        // use AABB algorithm
        vector<Eigen::Vector3d> points;
        Eigen::Vector3d robot_center(0.0, 0.0, (0.48+state[3])/2.0);
        Eigen::Vector3d robot_cube(0.22, 0.28, robot_center[2]+0.02);
        for (int i=-1; i<2; i+=2)
            for (int j=-1; j<2; j+=2)
                for (int k=-1; k<2; k+=2)
                {
                    points.push_back(robot_center + Eigen::Vector3d\
                                    (robot_cube[0]*i, robot_cube[1]*j, robot_cube[2]*k));
                }

        Eigen::Matrix2d R, RT;
        R << cos(state[2]) ,-sin(state[2]), \
             sin(state[2]) ,cos(state[2]) ;
        RT = R.transpose();
        Eigen::Vector2d t(state[0], state[1]);
        for (size_t i=0; i<8; i++)
        {
            points[i].block<2, 1>(0, 0) = R*points[i].head(2) + t; 
        }
        
        Eigen::Vector3d AABBmin(1e4, 1e4, 1e4), AABBmax(-1e4, -1e4, -1e4);
        Eigen::Vector3i AABBmin_idx, AABBmax_idx;
        for (size_t i=0; i<8; i++)
        {
            for (size_t j=0; j<3; j++)
            {
                if (points[i][j] < AABBmin[j])
                    AABBmin[j] = points[i][j];
                if (points[i][j] > AABBmax[j]);
                    AABBmax[j] = points[i][j];
            }
        }

        posToIndexWorld(AABBmin, AABBmin_idx);
        posToIndexWorld(AABBmax, AABBmax_idx);
        for (int i=AABBmin_idx(0); i<=AABBmax_idx(0); i++)
            for (int j=AABBmin_idx(1); j<=AABBmax_idx(1); j++)
                for (int k=AABBmin_idx(2); k<=AABBmax_idx(2); k++)
                {
                    Eigen::Vector3i idx(i, j, k);
                    Eigen::Vector3d pos;
                    if (isOccupancyWorld(idx) == 1)
                    {
                        indexToPosWorld(idx, pos);
                        Eigen::Vector4d p(pos[0], pos[1], pos[2], state[3]);
                        p.block<2, 1>(0, 0) = RT * (p.head(2) - t);
                        if (isOccupancy(p) == 1)
                        {
                            return false;
                        }
                    }
                    
                }
#else
        // using kd-tree search
        pcl::PointXYZ pointSearch;
        std::vector<int> pointIdx;
        std::vector<float> pointSquaredDistance;

        pointSearch.x = state[0];
        pointSearch.y = state[1];
        pointSearch.z = (0.48 + state[3])/2.0;
        kdtree.radiusSearch(pointSearch, 0.5, pointIdx, pointSquaredDistance);

        Eigen::Matrix2d R;
        R << cos(state[2]) , sin(state[2]), \
             -sin(state[2]), cos(state[2]);
        for (size_t i=0; i<pointIdx.size(); i++)
        {
            Eigen::Vector4d p(env_cloud->points[pointIdx[i]].x, \
                              env_cloud->points[pointIdx[i]].y, \
                              env_cloud->points[pointIdx[i]].z, state[3]);
            p.block<2, 1>(0, 0) = R * (p.head(2) - state.head(2));
            if (isOccupancy(p) == 1)
            {
                return false;
            }
        }
#endif
        return true;
    }

    inline bool MapManager::isHStateFree(const Eigen::Vector3d& state, double& h)
    {
#ifdef AABB
        // use AABB algorithm
        vector<Eigen::Vector3d> points;
        Eigen::Vector3d robot_center(0.0, 0.0, 0.32);
        Eigen::Vector3d robot_cube(0.22, 0.28, robot_center[2]+0.02);
        for (int i=-1; i<2; i+=2)
            for (int j=-1; j<2; j+=2)
                for (int k=-1; k<2; k+=2)
                {
                    points.push_back(robot_center + Eigen::Vector3d\
                                    (robot_cube[0]*i, robot_cube[1]*j, robot_cube[2]*k));
                }

        Eigen::Matrix2d R, RT;
        R << cos(state[2]) ,-sin(state[2]), \
             sin(state[2]) ,cos(state[2]) ;
        RT = R.transpose();
        Eigen::Vector2d t(state[0], state[1]);
        for (size_t i=0; i<8; i++)
        {
            points[i].block<2, 1>(0, 0) = R*points[i].head(2) + t; 
        }
        
        Eigen::Vector3d AABBmin(1e4, 1e4, 1e4), AABBmax(-1e4, -1e4, -1e4);
        Eigen::Vector3i AABBmin_idx, AABBmax_idx;
        for (size_t i=0; i<8; i++)
        {
            for (size_t j=0; j<3; j++)
            {
                if (points[i][j] < AABBmin[j])
                    AABBmin[j] = points[i][j];
                if (points[i][j] > AABBmax[j]);
                    AABBmax[j] = points[i][j];
            }
        }
        posToIndexWorld(AABBmin, AABBmin_idx);
        posToIndexWorld(AABBmax, AABBmax_idx);
        h = 0.16;
        double h_res = h*0.9;
        for (int i=AABBmin_idx(0); i<=AABBmax_idx(0); i++)
            for (int j=AABBmin_idx(1); j<=AABBmax_idx(1); j++)
                for (int k=AABBmin_idx(2); k<=AABBmax_idx(2); k++)
                {
                    Eigen::Vector3i idx(i, j, k);
                    Eigen::Vector3d pos;
                    if (isOccupancyWorld(idx) == 1)
                    {
                        indexToPosWorld(idx, pos);
                        Eigen::Vector4d p(pos[0], pos[1], pos[2], h);
                        p.block<2, 1>(0, 0) = RT * (p.head(2) - t);

                        double tmp_h = h;
                        for (; tmp_h>=0; tmp_h-=h_res)
                        {
                            p(3) = tmp_h;
                            if (isOccupancy(p) != 1)
                            {
                                h = tmp_h;
                                break;
                            }
                        }
                        if (tmp_h<0.0)
                        {
                            return false;
                        }
                    }
                }
#else
        // using kd-tree search
        pcl::PointXYZ pointSearch;
        std::vector<int> pointIdx;
        std::vector<float> pointSquaredDistance;

        pointSearch.x = state[0];
        pointSearch.y = state[1];
        pointSearch.z = 0.32;
        kdtree.radiusSearch(pointSearch, 0.5, pointIdx, pointSquaredDistance);

        Eigen::Matrix2d R;
        R << cos(state[2]) , sin(state[2]), \
             -sin(state[2]), cos(state[2]);
        h = 0.16;
        double h_res = h_resolution*0.9;
        for (size_t i=0; i<pointIdx.size(); i++)
        {
            Eigen::Vector4d p(env_cloud->points[pointIdx[i]].x, \
                              env_cloud->points[pointIdx[i]].y, \
                              env_cloud->points[pointIdx[i]].z, h);
            p.block<2, 1>(0, 0) = R * (p.head(2) - state.head(2));
            double tmp_h = h;
            for (; tmp_h>=0; tmp_h-=h_res)
            {
                p(3) = tmp_h;
                if (isOccupancy(p) != 1)
                {
                    h = tmp_h;
                    break;
                }
            }
            if (tmp_h<0.0)
            {
                return false;
            }
        }
#endif
        return true;
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

    inline void MapManager::posToIndexWorld(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
    {
        for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - world_origin(i)) * world_resolution_inv);
    }

    inline Eigen::Vector2i MapManager::posToIndexPlane(const Eigen::Vector2d& pos)
    {
        Eigen::Vector2i id;

        for (int i = 0; i < 2; ++i) id(i) = floor((pos(i) - world_origin(i)) * world_resolution_inv);

        return id;
    }

    inline void MapManager::indexToPosWorld(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
    {
        for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * world_resolution + world_origin(i);
    }

    inline int MapManager::toAddressWorld(const Eigen::Vector3i& id)
    {
        return id(0) * world_voxel_num(1)*world_voxel_num(2) + id(1) * world_voxel_num(2) + id(2);
    }

    inline int MapManager::toAddressWorld(const int& x, const int& y, const int& z)
    {
        return x * world_voxel_num(1)*world_voxel_num(2) + y * world_voxel_num(2) + z;
    }

    inline bool MapManager::isInMapWorld(const Eigen::Vector3d& pos)
    {
        if (pos(0) < world_min_boundary(0) + 1e-4 || \
            pos(1) < world_min_boundary(1) + 1e-4 || \
            pos(2) < world_min_boundary(2) + 1e-4     ) 
        {
            return false;
        }

        if (pos(0) > world_max_boundary(0) + 1e-4 || \
            pos(1) > world_max_boundary(1) + 1e-4 || \
            pos(2) > world_max_boundary(2) + 1e-4     ) 
        {
            return false;
        }

        return true;
    }

    inline bool MapManager::isInMapWorld(const Eigen::Vector3i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
        {
            return false;
        }

        if (idx(0) > world_voxel_num(0) - 1 || \
            idx(1) > world_voxel_num(1) - 1 || \
            idx(2) > world_voxel_num(2) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline bool MapManager::isInMapPlane(const Eigen::Vector3d& pos)
    {
        if (pos(0) < world_min_boundary(0) + 1e-4 || \
            pos(1) < world_min_boundary(1) + 1e-4     ) 
        {
            return false;
        }

        if (pos(0) > world_max_boundary(0) + 1e-4 || \
            pos(1) > world_max_boundary(1) + 1e-4     ) 
        {
            return false;
        }

        return true;
    }

    inline bool MapManager::isInMapPlane(const Eigen::Vector3i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0)
        {
            return false;
        }

        if (idx(0) > world_voxel_num(0) - 1 || \
            idx(1) > world_voxel_num(1) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline int MapManager::isOccupancyWorld(const Eigen::Vector3d& pos)
    {
        Eigen::Vector3i id;

        posToIndexWorld(pos, id);
            
        return isOccupancyWorld(id);
    }

    inline int MapManager::isOccupancyWorld(const Eigen::Vector3i& id)
    {
        if (!isInMapWorld(id))
            return -1;

        return int(world_buffer[toAddressWorld(id)]);
    }
    
};

#endif