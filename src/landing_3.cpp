/**
 * skeleton / example code for a node to do command and control of the pixhawk
 */

// includes
#include <math.h>
#include <iterator>
#include <random>
#include <iostream>
#include <ros/ros.h>
using namespace std;

// topic data
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
        float _flight_alt = 30.0f;		// desired flight altitude [m] AGL (above takeoff 0 height at the location of takeoff)

	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;
	geometry_msgs::TwistStamped _current_local_speed;

    geometry_msgs::PoseStamped _landing_Pos;

	// waypoint handling (example)
	int _wp_index = -1;
	int _n_waypoints = 1;
        float _target_U_lake = 20.0f; // This is the altitude desired w/r to lagunita frame
        float _target_E_lake=0.0f;
        float _target_N_lake=0.0f;
	float _target_v_E=0.0f;
	float _target_v_N=0.0f;
        float _target_v_U=0.0f;
	float _target_yaw=0.0f; // Pointing east by default


        // Gains of the PD
        float _Kp_x=0.1f;
        float _Kp_y=0.1f;
        float _Kp_z=0.1f;
        float _Kd_x=0.1f;
        float _Kd_y=0.1f;
        float _Kd_z=0.1f;

        // Maximum speed control for drone
        float _u_speed_max=7.0f;
        float _v_z_speed_max=3.0f;

        // Line handling I give one point of the line and an heading. Coordinates in the lake lagunita frame!
        float _line_E_lake=0.0f;
        float _line_N_lake=0.0f;
	double _pi = 3.1415926535897;
	float _line_heading=_pi/6;



        // Defining a fake camera lx,ly,lz represents the vector drone to landing site
        float lx=0.0f; // true east
        float ly=0.0f; // true north
        float lz=0.0f; //true up altidude







	// offset information
	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;

	// subscribers
	ros::Subscriber _state_sub;			// the current state of the pixhawk
	ros::Subscriber _local_pos_sub;		// local position information
	ros::Subscriber _local_speed_sub;		// local velocity information
	ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
	ros::Subscriber _mission_state_sub; // mission state
	// TODO: add subscribers here
ros::Subscriber _landing_pose_sub; // landing pose
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
	 * callback for the local speed computed by the pixhawk.
	* @param msg pose stamped message type
	 */
	void localSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

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
     void landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
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
	_local_speed_sub = _nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, &ControlNode::localSpeedCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::sensorMeasCallback, this);
        _mission_state_sub=_nh.subscribe<aa241x_mission::MissionState>("mission_state", 10,&ControlNode::missionStateCallback, this);
	// advertise the published detailed
 _landing_pose_sub =  _nh.subscribe<geometry_msgs::PoseStamped>("landing_pose", 1, &ControlNode::landingPoseCallback, this);
	// publish a PositionTarget to the `/mavros/setpoint_raw/local` topic which
	// mavros subscribes to in order to send commands to the pixhawk
	_cmd_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	// save the state locally to be used in the main loop
	_current_state = *msg;
}

void ControlNode::localSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	_current_local_speed=*msg;
}


void  ControlNode::landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    _landing_Pos = *msg;

    lx=_landing_Pos.pose.position.x;
    std::cout<<"lx in landingPoseCallback: "<<lx<<endl;
    ly=_landing_Pos.pose.position.y;
    lz=_landing_Pos.pose.position.z;
    //std::cout << "distance:" << lz //test tmr
}


void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	// save the current local position locally to be used in the main loop
	// TODO: account for offset to convert from PX4 coordinate to lake lag frame
	_current_local_pos = *msg;
        float _current_local_pos_E=_current_local_pos.pose.position.x;
        float _current_local_pos_N=_current_local_pos.pose.position.y;
        float _current_local_pos_U=_current_local_pos.pose.position.z;
        // Define random generator with Gaussian distribution to mimick uncertainty with camera data
//        const double mean = 0.0;
//        const double stddev = 0.01;
//        std::default_random_engine generator;
//        std::normal_distribution<double> dist(mean,stddev);
//        lx=-_current_local_pos_E+dist(generator);
//        ly=-_current_local_pos_N+dist(generator);
//        lz=-_current_local_pos_U+dist(generator);

	// TODO: make sure to account for the offset if desiring to fly in the Lake Lag frame
        // I assume e,n, u offset are component of the vector drone to lake
        float _current_local_pos_E_lake=_e_offset+_current_local_pos_E;
        float _current_local_pos_N_lake=_n_offset+_current_local_pos_N;
        float _current_local_pos_U_lake=_u_offset+_current_local_pos_U;


	// check to see if have completed the waypoint

    // First waypoint to mimic a take off while already in the air
        if (_wp_index == 0) {
                //_target_U_lake=_u_offset+_flight_alt;

		// check condition on being "close enough" to the waypoint
                if ((abs(_current_local_pos_U_lake - _target_U_lake) < 0.5) && (abs(_current_local_pos_N_lake - _target_N_lake)< 0.5) && (abs(_current_local_pos_E_lake - _target_E_lake)< 0.5) ) {
                    //std::cout<<"lx: "<<lx<<endl;
                        //ROS_INFO("PASSE AU MODE 1");
                        //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
			// increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
			_wp_index++;
                        _target_E_lake=_e_offset;
                        _target_N_lake=_n_offset;
                        _target_U_lake=_u_offset+2.5;
		}
        }

        if (_wp_index == 1) {
                //_target_U_lake=_u_offset+_flight_alt;

                // check condition on being "close enough" to the waypoint
                if ((abs(_current_local_pos_U_lake - _target_U_lake) < 0.5) && (abs(_current_local_pos_N_lake - _target_N_lake)< 0.5) && (abs(_current_local_pos_E_lake - _target_E_lake)< 0.5) && (lx!=0 || ly!=0 || lz!=0)) {
                        ROS_INFO("Goes to mode 2");
                        //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                        // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                        _wp_index++;
                        _target_E_lake=_current_local_pos_E_lake+lx;
                        _target_N_lake=_current_local_pos_N_lake-ly;
                        _target_U_lake=_current_local_pos_U_lake-lz;
                }
        }
        if (_wp_index == 2) {
                //_target_U_lake=_u_offset+_flight_alt;

                // check condition on being "close enough" to the waypoint
                if (abs(lz)<0.3) {
                        //ROS_INFO("PASSE AU MODE 1");
                        //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                        // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                        _wp_index++;

                }
        }





}

void ControlNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
	// TODO: use the information from the measurement as desired

	// NOTE: this callback is for an example of how to setup a callback, you may
	// want to move this information to a mission handling node
}//http://wiki.ros.org/roslaunch/XML

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




        // define the velocity control type mask
        // NOTE: type mask sets the fields to IGNORE
        uint16_t velocity_control_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

        // define the position control type mask
        // NOTE: type mask sets the fields to IGNORE
        uint16_t position_control_mask = (mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

        // default to velocity control so 0 means don't move
        cmd.type_mask = velocity_control_mask;


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
                        cmd.type_mask = velocity_control_mask;
                        vel.x = 0;
                        vel.y = 0;
                        vel.z = 0;


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

                // set the first waypoint, we want a mimic takeoff 15m above the current pos
		if (_wp_index < 0) {

			_wp_index = 0;

                        _target_E_lake=_e_offset;
                        _target_N_lake=_n_offset;
                        _target_U_lake=_u_offset+20;
                        _Kp_x=0.5f;
                        _Kp_y=0.5f;
                        _Kp_z=0.5f;
                        _Kd_x=0.5f;
                        _Kd_y=0.5f;
                        _Kd_z=0.5f;
		}

		// TODO: populate the control elements desired
		//




                float _current_local_pos_E=_current_local_pos.pose.position.x;
		float _current_local_pos_N=_current_local_pos.pose.position.y;
		float _current_local_pos_U=_current_local_pos.pose.position.z;

                if (_wp_index ==1) {
                    _Kp_x=0.1f;
                    _Kp_y=0.1f;
                    _Kp_z=0.1f;
                    _Kd_x=0.1f;
                    _Kd_y=0.1f;
                    _Kd_z=0.1f;
                    //std::cout<<"lx in wp_index1: "<<lx<<endl;
                }



                if(_wp_index==2) {
                 _Kp_x=0.1f;
                 _Kp_y=0.1f;
                 _Kp_z=0.1f;
                 _Kd_x=0.1f;
                 _Kd_y=0.1f;
                 _Kd_z=0.1f;
                 float _current_local_pos_E_lake=_e_offset+_current_local_pos_E;
                 float _current_local_pos_N_lake=_n_offset+_current_local_pos_N;
                 float _current_local_pos_U_lake=_u_offset+_current_local_pos_U;
                 _target_E_lake=_current_local_pos_E_lake+lx;
                 _target_N_lake=_current_local_pos_N_lake-ly;
                 _target_U_lake=_current_local_pos_U_lake-lz;
                }





                // Warning: we control everything in the local frame: need to define a control in the drone initial frame
                pos.x = _target_E_lake-_e_offset;
                pos.y = _target_N_lake-_n_offset;
                pos.z = _target_U_lake-_u_offset;


		float _current_local_speed_E=_current_local_speed.twist.linear.x;
		float _current_local_speed_N=_current_local_speed.twist.linear.y;
		float _current_local_speed_U=_current_local_speed.twist.linear.z;


                // We compute the error in the x and y directions in the local frame (same as lake by the way)
                float _epsilon_x=-_current_local_pos_E+pos.x;
                float _epsilon_y=-_current_local_pos_N+pos.y;
                float _epsilon_z=-_current_local_pos_U+pos.z;


                float _epsilon_speed_x=_current_local_speed_E;
                if(_epsilon_x>0 && _current_local_speed_E>0) {
                    _epsilon_speed_x=_current_local_speed_E;
                } else if(_epsilon_x>0 && _current_local_speed_E<0) {
                    _epsilon_speed_x=-_current_local_speed_E;
                } else if(_epsilon_x<0 && _current_local_speed_E<0) {
                    _epsilon_speed_x=_current_local_speed_E;
                } else {
                    _epsilon_speed_x=-_current_local_speed_E;
                }
                float _epsilon_speed_y=_current_local_speed_N;
                if(_epsilon_y>0 && _current_local_speed_N>0) {
                    _epsilon_speed_y=_current_local_speed_N;
                } else if(_epsilon_y>0 && _current_local_speed_N<0) {
                    _epsilon_speed_y=-_current_local_speed_N;
                } else if(_epsilon_y<0 && _current_local_speed_N<0) {
                    _epsilon_speed_y=_current_local_speed_N;
                } else {
                    _epsilon_speed_y=-_current_local_speed_N;
                }

                float _epsilon_speed_z=_current_local_speed_U;
                if(_epsilon_z>0 && _current_local_speed_U>0) {
                    _epsilon_speed_z=_current_local_speed_U;
                } else if(_epsilon_z>0 && _current_local_speed_U<0) {
                    _epsilon_speed_z=-_current_local_speed_U;
                } else if(_epsilon_z<0 && _current_local_speed_U<0) {
                    _epsilon_speed_z=_current_local_speed_U;
                } else {
                    _epsilon_speed_z=-_current_local_speed_U;
                }

                //float _current_slope=_epsilon_y/_epsilon_x;

                //float _projected_speed=(_current_local_speed_E+_a*_current_local_speed_N)/sqrt(1+pow(_current_slope,2));


                //compute the desired yaw to always face the target position
                _target_yaw=atan(_epsilon_y/_epsilon_x);
                if (_current_local_pos_E>pos.x) {
                    _target_yaw+=_pi;
                }

                vel.x=_Kp_x*_epsilon_x + _Kd_x*_epsilon_speed_x;
                vel.y=_Kp_y*_epsilon_y + _Kd_y*_epsilon_speed_y;
                vel.z=_Kp_z*_epsilon_z + _Kd_z*_epsilon_speed_z;


                if(abs(pow(vel.x,2)+pow(vel.y,2))>_u_speed_max) {
                    vel.x=_u_speed_max*cos(_target_yaw);
                    vel.y=_u_speed_max*sin(_target_yaw);
                }

                if(abs(vel.z)>_v_z_speed_max) {
                    vel.z=vel.z/abs(vel.z)*_v_z_speed_max;
                }



                if(_wp_index==3) {
                    vel.z=-0.01;
                    vel.x=0;
                    vel.y=0;
                }


                if (_wp_index ==0) {
                    _target_yaw=_pi/2;
                }

                if (_wp_index ==1) {
                    _target_yaw=_pi/2;
                }
                if(_wp_index==2) {
                    _target_yaw=_pi/2;
                }










		cmd.yaw=_target_yaw;

                cmd.type_mask =velocity_control_mask;

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
        ControlNode node(15.0f);

	// run the node
	return node.run();
}
