#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "boost/thread.hpp"

#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "octomap/octomap.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "fcl/collision_object.h"
//---OMPL
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <nav_msgs/Path.h>
#include <ompl/config.h>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
//---

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"
#include <mutex>          // std::mutex


using namespace Eigen;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

std::mutex mtx;           // mutex for critical section

typedef struct vector7d {
    Vector3d p;
    Vector4d q;
}vector7d;

class PLANNER {

	public:
		PLANNER();
		void run();
		void init_planner();
		void setStart(double x, double y, double z);
		void setGoal(double x, double y, double z);
		ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
		void plan(void);
		bool isStateValid(const ob::State *state);
		void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);
		bool plan_trajectory( vector7d pstart, vector7d pend, nav_msgs::Path & generated_path, vector<vector7d> & planned_path, float t);
		void path_monitor( );
		void octomap_cb(  const octomap_msgs::Octomap::ConstPtr &map );
		void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
		
	private:
		ros::NodeHandle _nh;
		
		//---Octomap
		octomap_msgs::Octomap _map;
		octomap::ColorOcTree* _tree;
		octomath::Vector3* _points;
		double _max_dist;
		bool _markers_searched;
		std::vector<Eigen::Vector3d> _markers;
		bool _check_map;
		bool _map_updated;
		bool _replan;
		bool _to_set_start;
		bool _in_planning;
		bool replan_flag;
		bool _smooth_path;
		ros::Subscriber _octree_sub;

		//---

    //---
    std::shared_ptr<fcl::CollisionGeometry> _Robot;
    std::shared_ptr<fcl::CollisionGeometry> _tree_obj;
    std::vector<fcl::CollisionObject*> _boxes;
    fcl::CollisionObject* _tree_cobj;
    fcl::OcTree* _fcl_tree;
    // construct the state space we are planning in
    ob::StateSpacePtr _space;
    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr _si;
    // create a problem instance
    ob::ProblemDefinitionPtr _pdef;
    // goal state
    og::PathGeometric* path_smooth = NULL;
};


PLANNER::PLANNER() {
	replan_flag = false;
	_in_planning = false;
	_map_updated = false;
	_smooth_path = false;
	
	_octree_sub = _nh.subscribe<octomap_msgs::Octomap>("/rtabmap/octomap_binary", 1, &PLANNER::octomapCallback, this); 

}




void PLANNER::setStart(double x, double y, double z) {
    _to_set_start = false;
    ob::ScopedState<ob::SE3StateSpace> start(_space);
    start->setXYZ(x,y,z);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    _pdef->clearStartStates();
    _pdef->addStartState(start);
}

void PLANNER::updateMap(std::shared_ptr<fcl::CollisionGeometry> map) {
    
    //if( !_in_planning ) {
        mtx.lock();
        _tree_obj = map;    
        mtx.unlock();
        
    //}
}


void PLANNER::init_planner() {
	_Robot = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.3, 0.3, 0.3));

	//fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
	//_tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);

	_space = ob::StateSpacePtr(new ob::SE3StateSpace());
	// create a start state
	ob::ScopedState<ob::SE3StateSpace> start(_space);
	// create a goal state
	ob::ScopedState<ob::SE3StateSpace> goal(_space);
	// set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
	bounds.setLow(0,-20);
	bounds.setHigh(0,20);
	bounds.setLow(1,-20);
	bounds.setHigh(1,20);
	bounds.setLow(2,-0.4);
	bounds.setHigh(2,2.8);
	_space->as<ob::SE3StateSpace>()->setBounds(bounds);
	// construct an instance of  space information from this state space
	_si = ob::SpaceInformationPtr(new ob::SpaceInformation(_space));
	start->setXYZ(0,0,0);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

	goal->setXYZ(0,0,0);


	goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// set state validity checking for this space
	_si->setStateValidityChecker(std::bind(&PLANNER::isStateValid, this, std::placeholders::_1 ));
	// create a problem instance
	_pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(_si));
	// set the start and goal states
	_pdef->setStartAndGoalStates(start, goal);
	// set Optimizattion objective
	_pdef->setOptimizationObjective(PLANNER::getPathLengthObjWithCostToGo(_si));
	_in_planning = false;

}



bool PLANNER::isStateValid(const ob::State *state) {

    // cast the abstract state type to the type we expect
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);


    mtx.lock();
    fcl::CollisionObject treeObj((_tree_obj));
    mtx.unlock();
    fcl::CollisionObject aircraftObject(_Robot);
    //return true;



    // check validity of state defined by pos & rot
    fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
    fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
    aircraftObject.setTransform(rotation, translation);
    fcl::CollisionRequest requestType(1,false,1,false);
    fcl::CollisionResult collisionResult;


    fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);



    return(!collisionResult.isCollision());

	//return true;
}


void PLANNER::setGoal(double x, double y, double z) {
    ob::ScopedState<ob::SE3StateSpace> goal(_space);
    goal->setXYZ(x,y,z);
    
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    _pdef->clearGoal();
    _pdef->setGoalState(goal);

    if(!_to_set_start)
        plan();    
}




ob::OptimizationObjectivePtr PLANNER::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si) {
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}




void PLANNER::plan(void) {

    // create a planner for the defined space
    ob::PlannerPtr plan(new og::InformedRRTstar(_si));

    // set the problem we are trying to solve for the planner
    plan->setProblemDefinition(_pdef);

    // perform setup steps for the planner
    plan->setup();

    // print the settings for this space
    _si->printSettings(std::cout);

    // print the problem settings
    _pdef->print(std::cout);
    _in_planning = true;
    // attempt to solve the problem within one second of planning time
    
    
    ob::PlannerStatus solved = plan->solve(2);
    ROS_WARN("SOLVED!");
    
    if (solved) {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        std::cout << "Found solution:" << std::endl;
        ob::PathPtr path = _pdef->getSolutionPath();
        og::PathGeometric* pth = _pdef->getSolutionPath()->as<og::PathGeometric>();
        //pth->printAsMatrix(std::cout);
        // print the path to screen
        // path->print(std::cout);
        //trajectory_msgs::MultiDOFJointTrajectory msg;
        //trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

        //msg.header.stamp = ros::Time::now();
        //msg.header.frame_id = "world";
        //msg.joint_names.clear();
        //msg.points.clear();
        //msg.joint_names.push_back("_Robot");
        
        nav_msgs::Path generated_path;

        generated_path.header.frame_id = "map";
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        generated_path.poses.clear();

        for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++) {

            const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
            const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            p.pose.position.x = pos->values[0];
            p.pose.position.y = pos->values[1];
            p.pose.position.z = pos->values[2];

            p.pose.orientation.x = rot->x;
            p.pose.orientation.y = rot->y;
            p.pose.orientation.z = rot->z;
            p.pose.orientation.w = rot->w;

						cout << "p: " << p << endl;
            generated_path.poses.push_back( p );
        }

      
        // Clear memory  
        _in_planning = false;
        _pdef->clearSolutionPaths();
        replan_flag = false;
    }
    else
        std::cout << "No solution found" << std::endl;


}

void PLANNER::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {

cout << "QUI" << endl;
	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<octomap::OcTree>(tree_oct));
	
	// Update the octree used for collision checking
	//updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));

	//_map_updated = true;
}

void PLANNER::run() {
	init_planner();

  while( !_map_updated) {
      ROS_WARN("Map not updated yet.");
      sleep(1);
      ros::spinOnce();
  }
  setStart(0,0,1);
	setGoal(1, 0, 1);
}



int main(int argc, char** argv)  {

	ros::init(argc, argv, "nav_node");
	PLANNER p;
	p.run();
	

	return 0;
}


