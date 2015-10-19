#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dji_ros/web_waypoint_receiveAction.h>
#include <iostream>

using namespace std;

typedef actionlib::SimpleActionServer<dji_ros::web_waypoint_receiveAction> ActionType;

void cb(const dji_ros::web_waypoint_receiveGoalConstPtr &goal, ActionType *ptr) {
	cout << "inside the cb" << endl;
	cout << (unsigned int) ptr << endl;
	cout << "id: " << goal->id << endl;
	//const dji_ros::waypoint[] wpl = goal->waypointList.waypointList;
	for(int i = 0; i < goal->waypointList.waypointList.size(); i++) {
		cout << goal->waypointList.waypointList[i] << endl;
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "web_waypoint_receive_action_test");
	ros::NodeHandle nh;

	ActionType* ptr_;

	dji_ros::web_waypoint_receiveFeedback feedback_;
	dji_ros::web_waypoint_receiveResult result_;

	ptr_ = new ActionType(nh, "/DJI_ROS/web_waypoint_receive_action", boost::bind(&cb, _1, ptr_), false);
	ptr_->start();

	ros::spin();

	return 0;
}
