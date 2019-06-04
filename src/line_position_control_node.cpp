/**
 * skeleton / example code for a node to do command and control of the pixhawk
 */

// includes
#include <math.h>

#include <ros/ros.h>

// topic data
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/SensorMeasurement.h>
#include <aa241x_mission/MissionState.h>

/**
 * class to contain the functionality of the controller node.
 */
class ControlNode {

public:

	/**
	 * example constructor.
	 * @param flight_alt the desired altitude for the takeoff point.
	 */
	ControlNode(float flight_alt);

	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();


private:


	// node handler
	ros::NodeHandle _nh;

	// TODO: add any settings, etc, here
	float _flight_alt = 20.0f;		// desired flight altitude [m] AGL (above takeoff)

	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;

	// waypoint handling (example)
	int _wp_index = -1;
	int _n_waypoints = 1;
	float _target_alt = 0.0f;
	float _target_E=0.0f;
	float _target_N=0.0f;
	float _target_v_E=0.0f;
	float _target_v_N=0.0f;
	float _target_yaw=0.0f; // Pointing east by default

	// Line handling I give one point of the line and an heading. I was not able to make this work
	// in the lake lagunita frame. I do everything in the local
	float _line_E=50.0f;
	float _line_N=-100.0f;
	double _pi = 3.1415926535897;
	float _line_heading=_pi/6;








	// offset information
	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;

	// subscribers
	ros::Subscriber _state_sub;			// the current state of the pixhawk
	ros::Subscriber _local_pos_sub;		// local position information
	ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
	ros::Subscriber _mission_state_sub; // mission state
	// TODO: add subscribers here

	// publishers
	ros::Publisher _cmd_pub;
	// TODO: recommend adding publishers for data you might want to log

	// callbacks

	/**
	 * callback for the current state of the pixhawk.
	 * @param msg mavros state message
	 */
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);

	/**
	 * callback for the local position and orientation computed by the pixhawk.
	 * @param msg pose stamped message type
	 */
	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	/**
	 * callback for the sensor measurement for the AA241x mission
	 * NOTE: you may end up wanting to move this to a separate mission handling
	 * node
	 * @param msg the AA241x sensor measurement
	 */
	void sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg);

	/**
	 * callback for the mission state for the AA241x mission
	 * this includes the offset information for the lake lag coordinate frame
	 * @param msg mission state
	 */
	void missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg);

	// TODO: add callbacks here

	// helper functions

	/**
	 * wait for the connection to the Pixhawk to be established.
	 */
	void waitForFCUConnection();


};


ControlNode::ControlNode(float flight_alt) :
_flight_alt(flight_alt)
{


	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ControlNode::stateCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ControlNode::localPosCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::sensorMeasCallback, this);
        //_mission_state_sub=_nh.subscribe<aa241x_mission::MissionState>("mission_state", 10,&ControlNode::missionStateCallback, this);
	// advertise the published detailed

	// publish a PositionTarget to the `/mavros/setpoint_raw/local` topic which
	// mavros subscribes to in order to send commands to the pixhawk
	_cmd_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	// save the state locally to be used in the main loop
	_current_state = *msg;
}

void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	// save the current local position locally to be used in the main loop
	// TODO: account for offset to convert from PX4 coordinate to lake lag frame
	_current_local_pos = *msg;
    float _current_local_pos_E=_current_local_pos.pose.position.x;
    float _current_local_pos_N=_current_local_pos.pose.position.y;
    float _current_local_pos_U=_current_local_pos.pose.position.z;

	// TODO: make sure to account for the offset if desiring to fly in the Lake Lag frame

	// check to see if have completed the waypoint

    // First waypoint to take off
	if (_wp_index == 0) {
		float current_alt = _current_local_pos.pose.position.z;

		// check condition on being "close enough" to the waypoint
		if (abs(current_alt - _target_alt) < 0.1) {
			// increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
			_wp_index++;
		}
	}

	// "Waypoint" that is not truly a waypoint just a step to compute the desired point to go on the line
	if (_wp_index == 1) {
			// I define the parameters of a line a direction and b offset
			float _a=tan(_line_heading);
			float _b=_line_N-_line_E*_a;
			// I then compute the crossing point
			float _crossing_E=_current_local_pos_E-_a*(_b-_current_local_pos_N)/(1+pow(_a,2));
			float _crossing_N=_a*_crossing_E+_b;


			// I find the heading
			_target_yaw=-_pi/2+_line_heading;
			if (_current_local_pos_N> _a*_current_local_pos_E) {
				_target_yaw=_pi/2+_line_heading;
			}
			_target_E=_crossing_E;
			_target_N=_crossing_N;
			_wp_index++;
		}

	// Waypoint follow the line for 100 m
	if (_wp_index == 2) {
		if (abs(_current_local_pos_E - _target_E) < 0.1 && abs(_current_local_pos_N - _target_N) < 0.1) {

				float _on_line_E=_target_E-100*cos(_line_heading);
				float _on_line_N=_target_N-100*sin(_line_heading);
				_target_E=_on_line_E;
				_target_N=_on_line_N;
				_target_yaw=_line_heading-_pi;
				_wp_index++;
			}
	}

	// Waypoint come home
		if (_wp_index == 3) {
			if (abs(_current_local_pos_E - _target_E) < 0.1 && abs(_current_local_pos_N - _target_N) < 0.1) {


					_target_E=0;
					_target_N=0;
					_wp_index++;
				}
		}

	//if (_wp_index == 2) {
				// check condition on being "close enough" to the waypoint retour a l
	//			if (abs(_current_local_pos_E - 20.0) < 0.1 && abs(_current_local_pos_N - 100.0) < 0.1) {
					// update the target altitude to land, and increment the waypoint
//					_target_alt = 0;
	//				_wp_index++;
		//		}
		//	}
}

void ControlNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
	// TODO: use the information from the measurement as desired

	// NOTE: this callback is for an example of how to setup a callback, you may
	// want to move this information to a mission handling node
}

void ControlNode::missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg) {
	// save the offset information
	_e_offset = msg->e_offset;
	_n_offset = msg->n_offset;
	_u_offset = msg->u_offset;
}


void ControlNode::waitForFCUConnection() {
	// wait for FCU connection by just spinning the callback until connected
	ros::Rate rate(5.0);
	while (ros::ok() && _current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
}


int ControlNode::run() {

	// wait for the controller connection
	waitForFCUConnection();
	ROS_INFO("connected to the FCU");



	// set up the general command parameters
	// NOTE: these will be true for all commands send
	mavros_msgs::PositionTarget cmd;
	cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;	// use the local frame

	// configure the type mask to command only position information
	// NOTE: type mask sets the fields to IGNORE
	// TODO: need to add a link to the mask to explain the value
	cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_VX |
		mavros_msgs::PositionTarget::IGNORE_VY |
		mavros_msgs::PositionTarget::IGNORE_VZ |
		mavros_msgs::PositionTarget::IGNORE_AFX |
		mavros_msgs::PositionTarget::IGNORE_AFY |
		mavros_msgs::PositionTarget::IGNORE_AFZ |
		mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

	// cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

	// the yaw information
	// NOTE: just keeping the heading north I THINK IT IS EAST
	cmd.yaw = 0;

	// the position information for the command
	// NOTE: this is defined in ENU
	geometry_msgs::Point pos;
	pos.x = 0;	// E
	pos.y = 0;	// N
	pos.z = 0;	// U

	// the velocity information for the command
	// NOTE: this is defined in ENU
	geometry_msgs::Vector3 vel;
	vel.x = 0;	// E
	vel.y = 0;	// N
	vel.z = 0;	// U

	// set the loop rate in [Hz]
	// NOTE: must be faster than 2Hz
	ros::Rate rate(10.0);

	// main loop
	while (ros::ok()) {

		// if not in offboard mode, just keep waiting until we are and if not
		// enabled, then keep waiting
		//
		// NOTE: need to be streaming setpoints in order for offboard to be
		// allowed, hence the publishing of an empty command
		if (_current_state.mode != "OFFBOARD") {
			// send command to stay in the same position
			// TODO: if doing position command in the lake lag frame, make
			// sure these values match the initial position of the drone!
			pos.x = 0;
			pos.y = 0;
			pos.z = 0;

			// timestamp the message and send it
			cmd.header.stamp = ros::Time::now();
			cmd.position = pos;
			cmd.velocity = vel;
			_cmd_pub.publish(cmd);

			// run the ros components
			ros::spinOnce();
			rate.sleep();
			continue;
		}

		// TODO: if drone is not armed at this point, need to send a command to
		// arm it
		//
		// NOTE: this can be done from either the callback or this main
		// function, so need to decide where I want to put it

		// at this point the pixhawk is in offboard control, so we can now fly
		// the drone as desired

		// set the first waypoint
		if (_wp_index < 0) {
			_wp_index = 0;
			_target_alt = _flight_alt;
		}

		// TODO: populate the control elements desired
		//




		pos.x = _target_E;
		pos.y = _target_N;
		pos.z = _target_alt;

		cmd.yaw=_target_yaw;



		// publish the command
		cmd.header.stamp = ros::Time::now();
		cmd.position = pos;
		cmd.velocity = vel;
		cmd.yaw=_target_yaw;

		_cmd_pub.publish(cmd);

		// remember need to always call spin once for the callbacks to trigger
		ros::spinOnce();
		rate.sleep();
	}

	// return  exit code
	return EXIT_SUCCESS;
}


int main(int argc, char **argv) {

	// initialize the node
	ros::init(argc, argv, "control_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings

	// create the node
	ControlNode node(10.0f);

	// run the node
	return node.run();
}
