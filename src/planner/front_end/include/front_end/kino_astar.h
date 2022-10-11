#ifndef _KINO_ASTAR_H_
#define _KINO_ASTAR_H_

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <algorithm>

#include "robo_map/robo_map_manager.h"

#define inf 1 >> 30
#define PI_X_2 6.283185307179586
#define CLOSE 'a'
#define OPEN 'b'
#define NOT_EXPAND 'c'

namespace ugv_planner
{
    class PathNode
    {
        public:
            Eigen::Vector3i index;
            Eigen::Vector3d state;
            Eigen::Vector2d input;
            double h;
            double g_score, f_score;
            double penalty_score;
            char node_state;
            int singul;
            PathNode* parent;
            PathNode(): parent(NULL), node_state(NOT_EXPAND), singul(0), h(0.0) {}
            ~PathNode() {}
    };
    typedef PathNode* PathNodePtr;

    class NodeComparator 
    {
        public:
            template <class NodePtr>
            bool operator()(NodePtr node1, NodePtr node2) 
            {
                return node1->f_score > node2->f_score;
            }
    };

    template <typename T>
    struct matrix_hash : std::unary_function<T, size_t> 
    {
        std::size_t operator()(T const& matrix) const 
        {
            size_t seed = 0;
            for (long int i = 0; i < matrix.size(); ++i) 
            {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    template <class NodePtr>
    class NodeHashTable 
    {
        private:
            std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> data_;

        public:
            NodeHashTable() {}
            ~NodeHashTable() {}

            void insert(Eigen::Vector3i idx,NodePtr node )
            {
                data_.insert(std::make_pair(idx, node));
            }

            NodePtr find(Eigen::Vector3i idx) 
            {
                auto iter = data_.find(idx);
                return iter == data_.end() ? NULL : iter->second;
            }

            void clear() { data_.clear(); }
    };


    class KinoAstar
    {
        private:
            MapManager::Ptr robo_map;
            std::vector<PathNodePtr> path_node_pool;
            NodeHashTable<PathNodePtr> expanded_nodes;
            std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set;
            std::vector<Eigen::Vector4d> front_end_path;
            std::vector<Eigen::Vector4d> shot_path;
            
            int allocate_num;
            double yaw_resolution, yaw_resolution_inv;
            double lambda_heu;
            double weight_r2;
            double weight_so2;
            double weight_v_change;
            double weight_w_change;
            double step_v;
            double step_w;
            double time_interval;
            double oneshot_range;
            double oneshot_interval;
            double collision_interval;

            //debug
            std::vector<Eigen::Vector3d> model_points;
            std::vector<visualization_msgs::Marker> model;

        public:
            KinoAstar() {}
            ~KinoAstar()
            {
                for (int i = 0; i < allocate_num; i++)
                    delete path_node_pool[i];
            }
            
            // debug
            ros::Publisher vis_pub;
            inline visualization_msgs::Marker getMarker(int id);
            void visWholeBodyPath();
            
            void setParam(ros::NodeHandle& nh);
            void init();
            inline void setEnvironment(const MapManager::Ptr& env);
            inline void stateToIndex(const Eigen::Vector3d& state, Eigen::Vector3i& idx); 
            inline int yawToIndex(const double &yaw);
            inline double normalizeAngle(const double &angle);
            inline double dAngle(const double &angle1, const double &angle2);
            inline double getHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
            inline void stateTransit(const Eigen::Vector3d &state0, Eigen::Vector3d &state1, \
                                    const Eigen::Vector2d &ctrl_input, const double& T);
            inline void asignShotTraj(const Eigen::Vector3d &state1, const Eigen::Vector4d &state_h);
            inline void retrievePath(PathNodePtr end_node);
            std::vector<Eigen::Vector4d> plan(const Eigen::Vector4d& start_state, const Eigen::Vector4d& end_state);

            typedef shared_ptr<KinoAstar> Ptr;
    };

    inline visualization_msgs::Marker KinoAstar::getMarker(int id)
    {
        visualization_msgs::Marker mk;
        mk.id               = id;
        mk.header.stamp     = ros::Time::now();
        mk.header.frame_id  = "world";
        mk.action           = visualization_msgs::Marker::ADD;
        mk.type             = visualization_msgs::Marker::CUBE;
        mk.ns               = "ugv";
        mk.color.r          = 1.0;
        mk.color.g          = 1.0;
        mk.color.b          = 1.0;
        mk.color.a          = 0.8;
        mk.pose.orientation.w = 1.0;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        return mk;
    }

    inline void KinoAstar::setEnvironment(const MapManager::Ptr& env)
    {
        this->robo_map = env;
        allocate_num = robo_map->world_voxel_num(0)*robo_map->world_voxel_num(1);
        for (int i = 0; i < allocate_num; i++)
        {
            path_node_pool.push_back(new PathNode());
        }
    }

    inline int KinoAstar::yawToIndex(const double &yaw)
    {
        double nor_yaw = normalizeAngle(yaw);
        int idx = floor((nor_yaw + M_PI) * yaw_resolution_inv);
        return idx;
    }
    
    inline void KinoAstar::stateToIndex(const Eigen::Vector3d& state, Eigen::Vector3i& idx)
    {
        idx.block<2, 1>(0, 0) = robo_map->posToIndexPlane(state.head(2));
        idx(2) = floor((normalizeAngle(state(2)) + M_PI) * yaw_resolution_inv);
    }

    inline double KinoAstar::normalizeAngle(const double &angle)
    {
        double nor_angle = angle;

        if (angle>=M_PI)
            nor_angle -= PI_X_2;

        if (angle<=-M_PI)
            nor_angle += PI_X_2;

        return nor_angle;
    }

    inline double KinoAstar::dAngle(const double &angle1, const double &angle2)
    {
        double da = angle1 - angle2;

        if (da > M_PI)
        {
            da -= PI_X_2;
        }
        else if (da < -M_PI)
        {
            da += PI_X_2;
        }

        return da;
    }

    inline double KinoAstar::getHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
    {
        return weight_r2 * (x1.head(2) - x2.head(2)).norm();
    }

    inline void KinoAstar::stateTransit(const Eigen::Vector3d &state0, Eigen::Vector3d &state1, \
                                        const Eigen::Vector2d &ctrl_input, const double& T)
    {
        double v = ctrl_input[0]; 
        double w = ctrl_input[1];
        double s = v * T;
        double y = w * T;
        
        if(fabs(w) > 1e-4)
        {
            double r = v / w;
            state1[0] = state0[0] + r*(sin(state0[2]+y)-sin(state0[2]));
            state1[1] = state0[1] - r*(cos(state0[2]+y)-cos(state0[2]));
            state1[2] = state0[2] + y;
            state1[2] = normalizeAngle(state1[2]);
        }
        else
        {
            state1[0] = state0[0] + s * cos(state0[2]);
            state1[1] = state0[1] + s * sin(state0[2]);
            state1[2] = state0[2];
        }
    }

    inline void KinoAstar::asignShotTraj(const Eigen::Vector3d &state_s, const Eigen::Vector4d &state_h)
    {
        // TODO: choose an optimal control solution
        shot_path.clear();
        Eigen::Vector2d direct_vec = state_h.head(2) - state_s.head(2);
        double direct = atan2(direct_vec(1), direct_vec(0));
        double dr = direct_vec.norm();

        Eigen::Vector4d p;
        double h = 0.0;
        p.block<3, 1>(0, 0) = state_s;
        
        if (dr > 1e-2)
        {
            double da = dAngle(direct, state_s(2));
            double wt = da>0.0 ? step_w*oneshot_interval*0.5 : -step_w*oneshot_interval*0.5;
            double fda = fabs(da);
            double fwt = fabs(wt);
            double vt = step_v * oneshot_interval * 0.5;
            double xt = vt*cos(direct);
            double yt = vt*sin(direct);

            while (fda > fwt)
            {
                p(2) += wt;
                p(2) = normalizeAngle(p(2));
                fda -= fwt;
                if (!robo_map->isHStateFree(p.head(3), h))
                {
                    shot_path.clear();
                    return;
                }
                p(3) = h;
                shot_path.push_back(p);
            }

            p(2) = direct;
            do
            {
                if (!robo_map->isHStateFree(p.head(3), h))
                {
                    shot_path.clear();
                    return;
                }
                p(3) = h;
                shot_path.push_back(p);
                p(0) += xt;
                p(1) += yt;
                dr -= vt;
            } while (dr > vt);

            double da2 = dAngle(state_h(2), direct);
            wt = da2>0.0 ? step_w*oneshot_interval*0.5 : -step_w*oneshot_interval*0.5;
            fda = fabs(da2);
            fwt = fabs(wt);
            while (fda > fwt)
            {
                p(2) += wt;
                p(2) = normalizeAngle(p(2));
                fda -= fwt;
                if (!robo_map->isHStateFree(p.head(3), h))
                {
                    shot_path.clear();
                    return;
                }
                p(3) = h;
                shot_path.push_back(p);
            }
        }
        else
        {
            double da = dAngle(state_h(2), state_s(2));
            double wt = da>0.0 ? step_w*oneshot_interval*0.5 : -step_w*oneshot_interval*0.5;
            double fda = fabs(da);
            double fwt = fabs(wt);

            while (fda > fwt)
            {
                p(2) += wt;
                p(2) = normalizeAngle(p(2));
                fda -= fwt;
                if (!robo_map->isHStateFree(p.head(3), h))
                {
                    shot_path.clear();
                    return;
                }
                p(3) = h;
                shot_path.push_back(p);
            }

        }
        
        shot_path.push_back(state_h);

        return;
    }

    inline void KinoAstar::retrievePath(PathNodePtr end_node)
    {
        int idx_interval = floor(time_interval / oneshot_interval);
        for (int i=shot_path.size()-1; i>=0; i-=idx_interval)
        {
            front_end_path.push_back(shot_path[i]);
        }

        PathNodePtr cur_node = end_node;
        Eigen::Vector4d state_h;
        state_h.block<3, 1>(0, 0) = cur_node->state;
        state_h(3) = cur_node->h;
        front_end_path.push_back(state_h);

        while (cur_node->parent != NULL)
        {
            cur_node = cur_node->parent;
            state_h.block<3, 1>(0, 0) = cur_node->state;
            state_h(3) = cur_node->h;
            front_end_path.push_back(state_h);
        }

        reverse(front_end_path.begin(), front_end_path.end());

        return;
    }
}

#endif