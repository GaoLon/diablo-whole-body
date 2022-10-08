#include <front_end/ompl_planner_mannager.h>

using namespace std;
using namespace Eigen;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ugv_planner
{

    OMPLPlanner::~OMPLPlanner(){ }

    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
    {
        return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    }

    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ob::Cost(2.0));
        return obj;
    }

    void OMPLPlanner::setEnvironment(const MapManager::Ptr& env)
    {
        this->robo_map = env;
    }

    void OMPLPlanner::visWholeBodyPath()
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

    bool OMPLPlanner::isStateValid(const ob::State *state)
    {
        const auto *se2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
        const auto *h = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

        ros::Time a = ros::Time::now();
        bool b = robo_map->isStateFree(Eigen::Vector4d(se2->getX(), se2->getY(), se2->getYaw(), h->values[0]));
        colli_check_time += (ros::Time::now() - a).toSec()*1000;
        check_times++;
        return b;
    }

    bool OMPLPlanner::isHStateValid(const ob::State *state)
    {
        const auto *se2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
        double h;

        return robo_map->isHStateFree(Eigen::Vector3d(se2->getX(), se2->getY(), se2->getYaw()), h);
    }

    void OMPLPlanner::propaGate(const oc::SpaceInformationPtr si, const ob::State *state, const oc::Control* control, const double duration, ob::State *result)
    {
        static double timeStep = 0.01;
        int nsteps = ceil(duration / timeStep);
        double dt = duration / nsteps;
        const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    
        ob::CompoundStateSpace::StateType& s = *result->as<ob::CompoundStateSpace::StateType>();
        ob::SE2StateSpace::StateType& se2 = *s.as<ob::SE2StateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType& vw = *s.as<ob::RealVectorStateSpace::StateType>(1);
        
        si->getStateSpace()->copyState(result, state);
        for(int i=0; i<nsteps; i++)
        {
            se2.setX(se2.getX() + dt * vw.values[0] * cos(se2.getYaw()));
            se2.setY(se2.getY() + dt * vw.values[0] * sin(se2.getYaw()));
            se2.setYaw(se2.getYaw() + dt * vw.values[1]);
            vw.values[0] = vw.values[0] + dt * u[0] * a_max;
            vw.values[1] = vw.values[1] + dt * u[1] * b_max;

            if (!si->satisfiesBounds(result))
                return;
        }
    }

    // state: x, y, yaw, v, w
    std::vector<Eigen::Vector4d> OMPLPlanner::kinoPlan(const Eigen::Matrix<double, 5, 1> start_state, const Eigen::Matrix<double, 5, 1> end_state)
    {
        front_end_path.clear();

        //set robot state space       
        auto SE2(std::make_shared<ob::SE2StateSpace>());
        auto vw(std::make_shared<ob::RealVectorStateSpace>(2));
        ob::StateSpacePtr stateSpace = SE2 + vw;
        stateSpace->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new MyProjection(stateSpace)));

        //set state space boundary
        ob::RealVectorBounds se2bounds(2);
        se2bounds.setLow(0, -1.0);
        se2bounds.setLow(1, -2.0);
        se2bounds.setHigh(0, 10.0);
        se2bounds.setHigh(1, 4.0);
        SE2->setBounds(se2bounds);

        ob::RealVectorBounds vwbounds(2);
        vwbounds.setLow(0, -1.5);
        vwbounds.setLow(1, -2.4);
        vwbounds.setHigh(0, 1.5);
        vwbounds.setHigh(1, 2.4);
        vw->setBounds(vwbounds);

        //set start state and goal state
        ob::ScopedState<> start(stateSpace);
        start[0] = start_state[0];
        start[1] = start_state[1];
        start[2] = start_state[2];
        start[3] = start_state[3];
        start[4] = start_state[4];

        ob::ScopedState<> goal(stateSpace);
        goal[0] = end_state[0];
        goal[1] = end_state[1];
        goal[2] = end_state[2];
        goal[3] = end_state[3];
        goal[4] = end_state[4];
        std::cout<<start_state.transpose()<<std::endl;
        std::cout<<end_state.transpose()<<std::endl;

        // set control space: a, beta
        oc::ControlSpacePtr cmanifold(std::make_shared<oc::RealVectorControlSpace>(stateSpace, 2));
        ob::RealVectorBounds cbounds(2);
        cbounds.setLow(0, -1.0);
        cbounds.setLow(1, -1.0);
        cbounds.setHigh(0, 1.0);
        cbounds.setHigh(1, 1.0);
        cmanifold->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

        // set state information
        auto si(std::make_shared<oc::SpaceInformation>(stateSpace, cmanifold));
        si->setPropagationStepSize(0.1);
        si->setMinMaxControlDuration(2, 3);
        si->setStateValidityChecker(std::bind(&OMPLPlanner::isHStateValid, this, std::placeholders::_1 ));
        si->setStatePropagator([this, si](const ob::State *state, const oc::Control* control,
        const double duration, ob::State *result)
        {
            this->propaGate(si, state, control, duration, result);
        });
        si->setup();
        
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(getPathLengthObjective(si));
    
        auto planner(std::make_shared<oc::PDST>(si));
        planner->as<oc::PDST>()->setProjectionEvaluator("myProjection");
        planner->setProblemDefinition(pdef);
        planner->setup();

        if (planner->ob::Planner::solve(0.1))
        {
            oc::PathControl* path = pdef->getSolutionPath()->as<oc::PathControl>();

            visualization_msgs::Marker marker;
            for (std::size_t idx = 0; idx < path->getStateCount(); idx++)
            {
                const ob::State* state = path->getState(idx);
                const auto *se2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
                
                front_end_path.push_back(Eigen::Vector4d(se2->getX(), se2->getY(), se2->getYaw(), 0.15));
            } 

            visWholeBodyPath(); 
        }

        return front_end_path;
    }

    // state: x, y, yaw, h
    std::vector<Eigen::Vector4d> OMPLPlanner::plan(const Eigen::Vector4d start_state, const Eigen::Vector4d end_state)
    {
        front_end_path.clear();
        
        //set robot state space       
        auto SE2(std::make_shared<ob::SE2StateSpace>());
        auto h(std::make_shared<ob::RealVectorStateSpace>(1));
        ob::StateSpacePtr stateSpace = SE2 + h;

        //set state space boundary
        ob::RealVectorBounds se2bounds(2);
        se2bounds.setLow(0, -1.0);
        se2bounds.setLow(1, -2.0);
        se2bounds.setHigh(0, 10.0);
        se2bounds.setHigh(1, 4.0);
        SE2->setBounds(se2bounds);

        ob::RealVectorBounds hbounds(1);
        hbounds.setLow(0.0);
        hbounds.setHigh(0.16);
        h->setBounds(hbounds);

        //set start state and goal state
        ob::ScopedState<> start(stateSpace);
        start[0] = start_state[0];
        start[1] = start_state[1];
        start[2] = start_state[2];
        start[3] = start_state[3];

        ob::ScopedState<> goal(stateSpace);
        goal[0] = end_state[0];
        goal[1] = end_state[1];
        goal[2] = end_state[2];
        goal[3] = end_state[3];

        //set space information
        ob::SpaceInformationPtr si(new ob::SpaceInformation(stateSpace));
        si->setStateValidityChecker(std::bind(&OMPLPlanner::isStateValid, this, std::placeholders::_1 ));
        si->setup();

        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(getPathLengthObjective(si));

        //set planner
        ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
        // ob::PlannerPtr optimizingPlanner(new og::RRTConnect(si));
        // ob::PlannerPtr optimizingPlanner(new og::PRM(si));
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // try solve
        ob::PlannerStatus solved;
        try
        { 
            solved = optimizingPlanner->solve(0.1);
        } 
        catch(ompl::Exception e) 
        { 
            ROS_WARN("Error occourred: %s", e.what()); 
        }

        // return path
        if (solved)
        {
            ROS_ERROR("check time=%lfms, check times=%d", colli_check_time, check_times);
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
            og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
            og::PathGeometric* path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            pathBSpline->smoothBSpline(*path_smooth, 2);
            path = path_smooth;

            visualization_msgs::Marker marker;
            for (std::size_t idx = 0; idx < path->getStateCount(); idx++)
            {
                const ob::State* state = path->getState(idx);
                const auto *se2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
                const auto *h = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
                
                front_end_path.push_back(Eigen::Vector4d(se2->getX(), se2->getY(), se2->getYaw(), h->values[0]));
            } 

            visWholeBodyPath(); 
        }
        
        return front_end_path;
    }

    void OMPLPlanner::init()
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

    void OMPLPlanner::setParam(ros::NodeHandle& nh)
    {
        nh.param("ompl/map_size_x", map_size_x, -1.0);
        nh.param("ompl/map_size_y", map_size_y, -1.0);
        nh.param("ompl/map_size_z", map_size_z, -1.0);
        nh.param("ompl/a_max", a_max, 0.5);
        nh.param("ompl/b_max", b_max, 3.0);
        std::cout<<"ab_max="<<a_max<<"  "<<b_max<<std::endl;
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/ompl_rrt/wholebody_marker", 0); 
    }

}