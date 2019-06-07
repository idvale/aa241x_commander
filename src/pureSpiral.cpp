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
using namespace std;

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
        float _flight_alt = 1.0f;		// desired flight altitude [m] AGL (above takeoff)

        // data
        mavros_msgs::State _current_state;
        geometry_msgs::PoseStamped _current_local_pos;
geometry_msgs::PoseStamped _landing_Pos;

        // waypoint handling (example)
        int _wp_index = 0;
        int _n_waypoints = 1;
        float _target_alt = 0.0f;
        float _target_Vd = 0.0f;
        float _target_V = 0.0f;
        float _target_Vin = 0.0f;
        float _target_x = 0.0f;
        float _target_y = 0.0f;
        float x0 = 0.0f;
        float y0 = 0.0f;
        float _target_Vx = 0.0f;
        float _target_Vy = 0.0f;
        float _target_Vz = 0.0f;

        float x0off = 0.0f;
        float y0off = 0.0f;
        float z0off = 0.0f;
        float dt = 0.05375f;
        float dtd = 0.5f;
        float xd = 0.0f;
        float yd = 0.0f;
        float ud = 0.0f;
        float vd = 0.0f;
        int count = 1;
        float dend = 0.0f;
        float omegad = 0.0f;
        float thetad = 0.0f;
        float omega = 0.0f;
        float theta = 0.0f;
        int curSpiral = 0;
        float dend0 = 0.0f;
        float den0 = 0.0f;
        float _target_Vind = 0.0f;
        float tol = 0.2f;
        float Tcor = 1.0f;
        float errX = 0.0f;
        float errY= 0.0f;
        float vcorrect = 0.0f;
        float xfinal = 0.0f;
        float yfinal = 0.0f;
        float lxstart = 0.0f;
        float lystart = 0.0f;
        float lzstart = 0.0f;


        // offset information
        float _e_offset = 0.0f;
        float _n_offset = 0.0f;
        float _u_offset = 0.0f;
        int state=0;

        // subscribers
        ros::Subscriber _state_sub;			// the current state of the pixhawk
        ros::Subscriber _local_pos_sub;		// local position information
        ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
        ros::Subscriber _mission_state_sub; // mission state
ros::Subscriber _landing_pose_sub; // landing pose
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
void landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        // helper functions

        /**
         * wait for the connection to the Pixhawk to be established.
         */
        void waitForFCUConnection();


};


ControlNode::ControlNode(float flight_alt) :
_flight_alt(flight_alt-z0off)

{

        // subscribe to the desired topics
        _state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ControlNode::stateCallback, this);
        _local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ControlNode::localPosCallback, this);
        _sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::sensorMeasCallback, this);
        _mission_state_sub = _nh.subscribe<aa241x_mission::MissionState>("mission_state", 10, &ControlNode::missionStateCallback, this);
_landing_pose_sub =  _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ControlNode::landingPoseCallback, this);
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
        x0off = _e_offset;
        y0off = _n_offset;
        z0off = _u_offset;
//        x0off = -123.01;
//        y0off = 160.31;
//        z0off = -11.21;
//        x0off = 20;
//        y0off = 10;
//        z0off = 10;
        float current_alt = _current_local_pos.pose.position.z;
        float current_x = _current_local_pos.pose.position.x;
        float current_y = _current_local_pos.pose.position.y;
        current_x += x0off;
        current_y += y0off;
        current_alt += z0off;
        //float den = sqrt(pow((current_x-x0),2)+pow((current_y-y0),2));
        float den = sqrt(pow((current_x),2)+pow((current_y),2));
        float dx = -10.0f;
        float dy  =10.0f;

//        std::cout<<"x0off: "<<x0off<<", y0off: "<<y0off<<", z0off: "<<z0off<<endl;

        // TODO: make sure to account for the offset if desiring to fly in the Lake Lag frame

        // check to see if have completed the waypoint
        // NOTE: for this case we only have a single waypoint
//        _target_alt = 2;

//        if (_wp_index == 0) {
//            if(current_x>dx){
//                _target_Vx = -1;
//                _target_Vy = 0;
//                _target_Vz = 0;
//                if(abs(current_y)>tol){
//                    _target_Vy = -current_y/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                    std::cout << "targetZ: " << _target_alt << ", currentZ:" << current_alt << ", w:" << _target_Vz << endl;
//                }
//            }
//            else if(current_x>2*dx){
//                _target_Vx = -2;
//                _target_Vy = 0;
//                _target_Vz = 0;
//                if(abs(current_y)>tol){
//                    _target_Vy = -current_y/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if(current_x>3*dx){
//                _target_Vx = -3;
//                _target_Vy = 0;
//                _target_Vz = 0;
//                if(abs(current_y)>tol){
//                    _target_Vy = -current_y/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if(current_x>4*dx){
//                _target_Vx = -2;
//                _target_Vy = 0;
//                _target_Vz = 0;
//                if(abs(current_y)>tol){
//                    _target_Vy = -current_y/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if(current_x>5*dx){
//                _target_Vx = -1;
//                _target_Vy = 0;
//                _target_Vz = 0;
//                if(abs(current_y)>tol){
//                    _target_Vy = -current_y/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }

//            else if (current_y< 1*dy) {
//                _target_Vx = 0;
//                _target_Vy = 1;
//                _target_Vz = 0;
//                if(abs(current_x-dx*5)>tol){
//                    _target_Vx = -(current_x-dx*5)/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if (current_y< 2*dy ) {
//                _target_Vx = 0;
//                _target_Vy = 2;
//                _target_Vz = 0;
//                if(abs(current_x-dx*5)>tol){
//                    _target_Vx = -(current_x-dx*5)/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if (current_y< 3*dy ) {
//                _target_Vx = 0;
//                _target_Vy = 3;
//                _target_Vz = 0;
//                if(abs(current_x-dx*5)>tol){
//                    _target_Vx = -(current_x-dx*5)/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if (current_y< 4*dy) {
//                _target_Vx = 0;
//                _target_Vy = 2;
//                _target_Vz = 0;
//                if(abs(current_x-dx*5)>tol){
//                    _target_Vx = -(current_x-dx*5)/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if (current_y< 5*dy) {
//                _target_Vx = 0;
//                _target_Vy = 1;
//                _target_Vz = 0;
//                if(abs(current_x-dx*5)>tol){
//                    _target_Vx = -(current_x-dx*5)/Tcor;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            if(current_x<(5*dx) && (current_y>(5*dy))){
//                _wp_index++;
//                xfinal = current_x;
//                yfinal = current_y;
//                std::cout << "xfinal: " << xfinal << ", yfinal:" << yfinal << endl;
//            }
//}

//            if(_wp_index ==1){
//            if (current_x<(5*dx) && (current_y>(5*dy))){
//                _target_Vx = 3;
//                _target_Vy = -3;
//                _target_Vz = 0;
//                std::cout << "xfinal: " << xfinal << ", yfinal:" << yfinal << endl;
//                std::cout << "current_x: " << current_x << ", current_x:" << current_x << endl;
//                std::cout << "errXx: " << (xfinal+dt*_target_Vx - current_x) << ", errXy:" << (yfinal+dt*_target_Vy - current_y) << endl;
//                if(abs( (xfinal+dt*_target_Vx - current_x) )>tol){
//                    _target_Vx = ((xfinal+dt*_target_Vx - current_x)+2*_target_Vx)/Tcor/2;
//                }
//                if(abs( (yfinal+dt*_target_Vy - current_y) )>tol){
//                    _target_Vy = ((yfinal+dt*_target_Vy - current_y)+2*_target_Vy)/Tcor/2;
//                }
//                if(abs(current_alt-_target_alt)>tol){
//                    _target_Vz = -(current_alt-_target_alt)/Tcor;
//                }
//            }
//            else if (abs(current_x)<(1) || (abs(current_y)<(1))){
//                _target_Vx = 0;
//                _target_Vy = 0;
//                _target_Vz = 0;
//            }
//            }



//            std::cout << "u: " << _target_Vx << ", v:" << _target_Vy << ", w:" << _target_Vz << endl;
//            std::cout << "currentX: " << current_x << ", currentY:" << current_y << ", currentZ:" << current_alt << endl;

               if(_wp_index == 0){

                _target_alt = 20;
                _target_x = 0;
                _target_y = 25;
                _target_Vx = 10*(_target_x-current_x)/den;
                _target_Vy = 10*(_target_y-current_y)/den;

                _target_Vz=0;
                if(abs(current_alt-_target_alt)>tol){
                    _target_Vz = -(current_alt-_target_alt)/Tcor;
                }

                // check condition on being "close enough" to the waypoint
                if (abs(current_x - _target_x) < 1 || abs(current_y - _target_y) < 1) {
                        // update the target altitude to land, and increment the waypoint
                        _wp_index++;
                        xd = current_x;
                        yd = current_y;
                        dend0 = sqrt(pow(xd,2)+pow(yd,2));
                        den0 = dend0;
                }
               }


        if (_wp_index == 1) {
//            std::cout<<"hello"<<endl;
            _target_Vd = 5;
            _target_V = 5;
            curSpiral = floor(thetad/2/3.14159)+1;
            if(thetad<(curSpiral*2*3.14159 - 0.1)){
                    _target_Vind = _target_Vd/2/3.14159*log((dend0+2*25/2)/dend0);
                    _target_Vin = _target_V/2/3.14159*log((den0+2*25/2)/den0);
                    count++;
            }
             else {
                 dend0 = dend;
                 den0 = den;
              }

            dend = sqrt(pow(xd,2)+pow(yd,2));
//            errX = dend-den;
//            _target_Vin = _target_Vind;
//            if(abs(errX)>tol){
//                _target_Vin = (_target_Vind+2*errX)/Tcor;
//                std::cout << "errX: " << errX << ", Vind" << _target_Vind <<  ", Vin" << _target_Vin << endl;
//            }

//            errY= thetad-theta;
//            _target_V = _target_Vd;
//            if(abs(errX)>tol){
//                _target_V = (_target_Vd+2*errY)/Tcor;
//                std::cout << "errY: " << errY<< ", Vd" << _target_Vd <<  ", V" << _target_V << endl;
//            }

//            if(abs(current_alt-_target_alt)>tol){
//                _target_Vz = -(current_alt-_target_alt)/Tcor;
//            }

//            _target_V = _target_Vd;
//            _target_Vin = _target_Vind;
//            _target_Vx = _target_V *(current_y-y0)/den + _target_Vin * (current_x-x0)/den;
//            _target_Vy = _target_V * (x0 - current_x)/den + _target_Vin * (current_y-y0)/den;

            ud = _target_Vd *(yd-y0)/dend + _target_Vind * (xd-x0)/dend;
            vd = _target_Vd * (x0 - xd)/dend + _target_Vind * (yd-y0)/dend;
            xd = xd+ ud*dtd;
            yd = yd+ vd*dtd;

            _target_Vx = ud;
            _target_Vy = vd;
            errX = xd - current_x;
            if(abs(errX)>tol){
               _target_Vx = (ud+2*errX)/Tcor;
             }
            errY= yd - current_y;
            if(abs(errY)>tol){
               _target_Vy = (vd+2*errY)/Tcor;
             }

            _target_Vz=0;
            if(abs(current_alt-_target_alt)>tol){
                _target_Vz = -(current_alt-_target_alt)/Tcor;
            }

            std::cout << "errX: " << errX << ", ud: " << ud <<  ", u: " << _target_Vx << ", w: " << _target_Vz <<endl;
            std::cout << "errY: " << errX << ", vd: " << vd <<  ", v: " << _target_Vy << endl;

            omegad = _target_Vd/dend;
            thetad = thetad + omegad*dtd;

            omega = _target_V/den;
            theta = theta + omega*dt;


//            std:cout<<"theta"<<theta<<endl;
            if(abs(theta-4*3.14159)<0.1){
                _wp_index++;
            }
            std::cout << "xd: " << xd << ", x: " << current_x <<  ", yd: " << yd <<  ", y: " <<  current_y << ", count: " << count<< endl;
        }



        if(_wp_index==2){
            _target_x = x0off;
            _target_y = y0off;
            _target_Vx = 10*(_target_x-current_x)/den;
            _target_Vy = 10*(_target_y-current_y)/den;
            _target_Vz=0;
            if(abs(current_alt-_target_alt)>tol){
                _target_Vz = -(current_alt-_target_alt)/Tcor;
            }
            std::cout << "u: " << _target_Vx << ", v: " << _target_Vy <<endl;
            if (abs(current_x - _target_x) < 1 || abs(current_y - _target_y) < 1) {
                    _wp_index++;
                    _target_x = x0off;
                    _target_y = y0off;
                    _target_alt = -z0off+15;
            }

        }


        if(_wp_index==3){
            _target_Vx = 10*(_target_x-current_x)/den;
            _target_Vy = 10*(_target_y-current_y)/den;
            _target_Vz = 10*(_target_alt-current_alt)/den;
            std::cout << "x: " << current_x << ", y: " << current_y<< ", z: " << current_alt <<", zd: " << _target_alt <<endl;
            std::cout << "u: " << _target_Vx << ", v: " << _target_Vy<< ", w: " << _target_Vz <<endl;
            if (abs(current_alt - _target_alt) < 0.1) {
                    _wp_index++;
                    lxstart = current_x;
                    lystart = current_y;
                    lzstart = current_alt;
            }
        }



        if(_wp_index == 4){
            _target_Vx = 0;
            _target_Vy = 0;
            _target_Vz = 0;
//            _target_x = current_x - lx;
//            _target_y = current_y + ly;
//            _target_alt = current_alt - lz;
//            _target_Vx = 0.5*(_target_x)/den;
//            _target_Vy = 0.5*(_target_y)/den;
//            _target_Vz = 0.5*(_target_alt)/den;
//            if (abs(current_alt - _target_alt) < 0.1) {
//                    _wp_index++;
//            }
        }


        if(_wp_index == 5){
                        _target_Vx = 0;
                        _target_Vy = 0;
                        _target_Vz = 0;
        }





}


void ControlNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
        // TODO: use the information from the measurement as desired

        // NOTE: this callback is for an example of how to setup a callback, you may
        // want to move this information to a mission handling node
}

void ControlNode::missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg) {
        // save the offset information
        state = msg->mission_state;
        _e_offset = msg->e_offset;
        _n_offset = msg->n_offset;
        _u_offset = msg->u_offset;
        //std::cout << "offset" <<_e_offset << "state"<<state;
}


void  ControlNode::landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    _landing_Pos = *msg;
    //////////////////////////////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/////////////////////////////////////
    float lx=_landing_Pos.pose.position.x;
    float ly=_landing_Pos.pose.position.y;
    float lz=_landing_Pos.pose.position.z;
//    std::cout << "distance:" << lz //test tmr
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
        cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

        //cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

        // the yaw information
        // NOTE: just keeping the heading north
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

                // TODO: populate the control elements desired
                //
                // in this case, just asking the pixhawk to takeoff to the _target_alt
                // height
                vel.x = _target_Vx;
                vel.y = _target_Vy;
                vel.z = _target_Vz;
//                pos.z = _target_alt;

                // publish the commandrrt
                cmd.header.stamp = ros::Time::now();
                cmd.position = pos;
                cmd.velocity = vel;
                _cmd_pub.publish(cmd);



                                // remember need to always call spin once for the callbacks to trigger
                                ros::spinOnce();
                                rate.sleep();
                        }

                        // return  exit code
                        return EXIT_SUCCESS;
                }


                int main(int argc, char **argv) {

                        // initialize th enode
                        ros::init(argc, argv, "control_node");

                        // get parameters from the launch file which define some mission
                        // settings
                        ros::NodeHandle private_nh("~");
                        // TODO: determine settings

                        // create the node
                        ControlNode node(20.0f);

                        // run the node
                        return node.run();
                }
