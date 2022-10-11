#include <front_end/kino_astar.h>

namespace ugv_planner
{
    void KinoAstar::visWholeBodyPath()
    {
        visualization_msgs::MarkerArray ma;
        int n = model.size();
        
        for (size_t i=0; i<front_end_path.size(); i++)
        {
            Eigen::Vector4d way_point = front_end_path[i];
            Eigen::Matrix3d R;
            R << cos(way_point[2]), -sin(way_point[2]), 0.0,\
                 sin(way_point[2]), cos(way_point[2]) , 0.0,\
                 0.0              , 0.0               , 1.0;
            Eigen::Quaterniond q(R);
            for (size_t j=0; j<model.size(); j++)
            {
                Eigen::Vector3d odom = R*model_points[j];
                model[j].pose.orientation.w = q.w(); 
                model[j].pose.orientation.x = q.x(); 
                model[j].pose.orientation.y = q.y();
                model[j].pose.orientation.z = q.z();
                model[j].pose.position.x = way_point[0] + odom[0];
                model[j].pose.position.y = way_point[1] + odom[1];
                model[j].id = model[j].id + i*n;
                if (j==0)
                    model[j].pose.position.z = way_point[3] + odom[2];
                ma.markers.push_back(model[j]);
            }
        }

        vis_pub.publish(ma);
        ros::Duration(0.5).sleep(); 
    }

    std::vector<Eigen::Vector4d> KinoAstar::plan(const Eigen::Vector4d& start_state, const Eigen::Vector4d& end_state)
    {
        // reset
        int use_node_num = 0;
        int iter_num = 0;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set.swap(empty_queue);
        front_end_path.clear();
        expanded_nodes.clear();
        for (int i = 0; i < use_node_num; i++)
        {
            PathNodePtr node = path_node_pool[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }
        
        Eigen::Vector3d end_pt(end_state.head(3));
        if (!robo_map->isStateFree(end_state))
        {
            ROS_ERROR("goal is not free!!!");
            return front_end_path;
        }
        ros::Time t0 = ros::Time::now();
        PathNodePtr cur_node = path_node_pool[0];
        cur_node->parent = NULL;
        cur_node->state = start_state.head(3);
        cur_node->state(2) = normalizeAngle(cur_node->state(2));
        stateToIndex(cur_node->state, cur_node->index);
        cur_node->g_score = 0.0;
        cur_node->input = Eigen::Vector2d(0.0, 0.0);
        cur_node->f_score = lambda_heu * getHeu(cur_node->state, end_pt);
        cur_node->node_state = OPEN;

        open_set.push(cur_node);
        use_node_num += 1;
        expanded_nodes.insert(cur_node->index, cur_node);
        bool is_init = false;

        if(cur_node->singul == 0)
            is_init = true;
        
        while (!open_set.empty())
        {
            cur_node = open_set.top();

            if((cur_node->state.head(2) - end_pt.head(2)).norm() < oneshot_range && is_init)
            {
        // ROS_INFO("ggg");
                ros::Time t1 = ros::Time::now();
                asignShotTraj(cur_node->state, end_state);
        // ROS_INFO("ggok");
                if (!shot_path.empty())
                {
                    std::cout << "one-shot time: " << (ros::Time::now()-t1).toSec()*1000 << " ms"<<std::endl;
                    std::cout << "all planning time: " << (ros::Time::now()-t0).toSec()*1000 << " ms"<<std::endl;
                    retrievePath(cur_node);
                    visWholeBodyPath();
                    return front_end_path;
                }
            }
        // ROS_INFO("4");

            open_set.pop();
            cur_node->node_state = CLOSE;
            iter_num += 1;

            Eigen::Vector3d cur_state = cur_node->state;
            Eigen::Vector3d pro_state;
            Eigen::Vector2d ctrl_input;
            std::vector<Eigen::Vector2d> inputs;

            if(!is_init )
            {
                //TODO
                is_init = true;
            }
            else
            {
                for (double v = 0; v <= step_v + 1e-3; v += 0.5*step_v)
                {
                    for (double w = -step_w; w <= step_w + 1e-3; w += 0.2*step_w)
                    {
                        ctrl_input << v, w;
                        inputs.push_back(ctrl_input);
                    }
                }
            }

            for (size_t i=0; i<inputs.size(); i++)
            {
        // ROS_INFO("5");
                Eigen::Vector2d input = inputs[i];
                stateTransit(cur_state, pro_state, input, time_interval);
        // ROS_INFO("6");

                if (!robo_map->isInMapPlane(cur_state))
                {
                    std::cout << "[Kino Astar]: out of map range" << std::endl;
                    continue;
                }
        // ROS_INFO("7");

                Eigen::Vector3i pro_id;
                PathNodePtr pro_node;

                stateToIndex(pro_state, pro_id);
                pro_node = expanded_nodes.find(pro_id);

                if (pro_node != NULL && pro_node->node_state == CLOSE)
                {
                    continue;
                }
        // ROS_INFO("8");

                Eigen::Vector3d xt;
                double h = 0.0;
                bool isfree = true;
                for (double t = collision_interval; t <= time_interval+1e-3; t+=collision_interval)
                {
        // ROS_INFO("s");
                    stateTransit(cur_state, xt, input, t);
                    isfree = robo_map->isHStateFree(xt, h);
        // ROS_INFO("b");

                    if (!isfree)
                        break;
                }
                if (!isfree)  
                    continue;
        // ROS_INFO("9");

                double tmp_g_score = 0.0;
                double tmp_f_score = 0.0;
                tmp_g_score += weight_r2 * input(0) * time_interval;
                tmp_g_score += weight_so2 * fabs(input(1)) * time_interval;
                tmp_g_score += weight_v_change * std::fabs(input(0)-cur_node->input(0));
                tmp_g_score += weight_w_change * std::fabs(input(1)-cur_node->input(1));
                tmp_g_score += cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu * getHeu(pro_state, end_pt);

                if (pro_node == NULL)
                {
        // ROS_INFO("10");
                    pro_node = path_node_pool[use_node_num];
                    pro_node->index = pro_id;
                    pro_node->state = pro_state;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->input = input;
                    pro_node->h = h;
                    pro_node->parent = cur_node;
                    pro_node->node_state = OPEN;
                    open_set.push(pro_node);

                    expanded_nodes.insert(pro_id, pro_node);
                    use_node_num ++;

                    if (use_node_num == allocate_num)
                    {
                        std::cout << "run out of memory." << std::endl;
                        return front_end_path;
                    }
                }
                else if (pro_node->node_state == OPEN)
                {
        // ROS_INFO("11");
                    if (tmp_g_score < pro_node->g_score)
                    {
                        pro_node->index = pro_id;
                        pro_node->state = pro_state;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = input;
                        pro_node->h = h;
                        pro_node->parent = cur_node;
                    }
                }
            }
        }

        std::cout << "Kino Astar Failed, No path!!!" << std::endl;

        return front_end_path;
    }

    void KinoAstar::init()
    {
        visualization_msgs::Marker mk;

        mk = getMarker(0);
        mk.pose.position.x = 0.0;
        mk.pose.position.y = 0.0;
        mk.pose.position.z = 0.28;
        mk.scale.x = 0.4;
        mk.scale.y = 0.3;
        mk.scale.z = 0.4;
        model.push_back(mk);
        model_points.push_back(Eigen::Vector3d(mk.pose.position.x,\
                                                mk.pose.position.y,\
                                                mk.pose.position.z));

        mk = getMarker(1);
        mk.pose.position.x = 0.0;
        mk.pose.position.y = 0.2;
        mk.pose.position.z = 0.15;
        mk.scale.x = 0.2;
        mk.scale.y = 0.1;
        mk.scale.z = 0.3;
        model.push_back(mk);
        model_points.push_back(Eigen::Vector3d(mk.pose.position.x,\
                                                mk.pose.position.y,\
                                                mk.pose.position.z));

        mk = getMarker(2);
        mk.pose.position.x = 0.0;
        mk.pose.position.y = -0.2;
        mk.pose.position.z = 0.15;
        mk.scale.x = 0.2;
        mk.scale.y = 0.1;
        mk.scale.z = 0.3;
        model.push_back(mk);
        model_points.push_back(Eigen::Vector3d(mk.pose.position.x,\
                                                mk.pose.position.y,\
                                                mk.pose.position.z));
    }

    void KinoAstar::setParam(ros::NodeHandle& nh)
    {
        nh.param("kino_astar/yaw_resolution", yaw_resolution, 3.15);
        nh.param("kino_astar/lambda_heu", lambda_heu, 1.0);
        nh.param("kino_astar/weight_r2", weight_r2, 1.0);
        nh.param("kino_astar/weight_so2", weight_so2, 1.0);
        nh.param("kino_astar/weight_v_change", weight_v_change, 0.0);
        nh.param("kino_astar/weight_w_change", weight_w_change, 0.0);
        nh.param("kino_astar/step_v", step_v, 1.5);
        nh.param("kino_astar/step_w", step_w, 2.4);
        nh.param("kino_astar/time_interval", time_interval, 0.5);
        nh.param("kino_astar/oneshot_range", oneshot_range, 3.0);
        nh.param("kino_astar/oneshot_interval", oneshot_interval, 0.1);
        nh.param("kino_astar/collision_interval", collision_interval, 0.1);
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/kino_astar/wholebody_marker", 0); 
    }
}