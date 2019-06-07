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
		std::vector<int> ID;
                std::vector<int> ID2detect;
                std::vector<int> IDdetected;
                std::vector<float> xB2detect;
                std::vector<float> yB2detect;
                std::vector<float> xBFinal;
                std::vector<float> yBFinal;
                std::vector<float> xBcount;
                std::vector<float> yBcount;
        std::vector<float> p_y;
        std::vector<float> p_x;
        
        int _wp_index = 0;
        int _n_waypoints = 1;
        float _target_alt = 0.0f;
        float _target_Vd = 3.0f;
        float _target_V = 3.0f;
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
        float dtd = 0.2f;
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
        float tol = 0.1f;
        float Tcor = 1.0f;
        float errX = 0.0f;
        float errY= 0.0f;
        float vcorrect = 0.0f;
        float xfinal = 0.0f;
        float yfinal = 0.0f;
        float lxstart = 0.0f;
        float lystart = 0.0f;
        float lzstart = 0.0f;
        float lx = 0.0f;
        float ly = 0.0f;
        float lz = 0.0f;
        float beaconXloc= 0.0f;
        float beaconYloc = 0.0f;
        float beaconStartx = 0.0f;
        float beaconStarty = 0.0f;
        float xBeacond = 0.0f;
        float yBeacond = 0.0f;
        float uBeacond = 0.0f;
        float vBeacond = 0.0f;
        float denBd = 0.0f;
        float omegaBd = 0.0f;
        float thetaBd = 0.0f;
        int numBeacon = 100;
        float diaDetect = 0.0f;
        float vinBeacon = 0.0f;
        float curBXloc= 0.0f;
        float curBYloc = 0.0f;
        float beaconXlocReadIn= 0.0f;
        float beaconYlocReadIn = 0.0f;
        int ii = 0;
        int curID = 0;
        double vSmallCircle = 1.0f;
        double v2SmallCircle = 3.5f;
        float sumBXloc= 0.0f;
        float sumBYloc = 0.0f;
        float diaSmallCircle = 8.0f;
        bool countTrue = false;
        float beaconXlocCount= 0.0f;
        float beaconYlocCount = 0.0f;

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
        ros::Publisher _cmd_pub;z
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
         * @param msg mission state<<endl;
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
        _sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 1000, &ControlNode::sensorMeasCallback, this);
        _mission_state_sub = _nh.subscribe<aa241x_mission::MissionState>("mission_state", 10, &ControlNode::missionStateCallback, this);
_landing_pose_sub =  _nh.subscribe<geometry_msgs::PoseStamped>("landing_pose", 1, &ControlNode::landingPoseCallback, this);
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

//        std::cout<<"x0off: "<<x0off<<", y0off: "<<y0off<<", z0off: "<<z0off<<endl;

        // TODO: make sure to account for the offset if desiring to fly in the Lake Lag frame

        // check to see if have completed the waypoint
        // NOTE: for this case we only have a single waypoint

               if(_wp_index == 0){
                _target_alt = 30;
                diaDetect = 5/7*_target_alt+28.75-5;

                _target_x = 0;
                _target_y = diaDetect;
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
                        ID2detect.push_back(100);
                        IDdetected.push_back(100);
                        xB2detect.push_back(100);
                        yB2detect.push_back(100);
                        xd = current_x;
                        yd = current_y;
                        dend0 = sqrt(pow(xd,2)+pow(yd,2));
                        den0 = dend0;
                }
               }


        if (_wp_index == 1) {
            curSpiral = floor(thetad/2/M_PI)+1;
            if(thetad<(curSpiral*2*M_PI - 0.1)){
                    _target_Vind = _target_Vd/2/M_PI*log((dend0+diaDetect)/dend0);
                    _target_Vin = _target_V/2/M_PI*log((den0+diaDetect)/den0);
            }
             else {
                 dend0 = dend;
                 den0 = den;
              }

            dend = sqrt(pow(xd,2)+pow(yd,2));
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

//            std::cout << "errX: " << errX << ", ud: " << ud <<  ", u: " << _target_Vx << ", w: " << _target_Vz <<endl;
//            std::cout << "errY: " << errX << ", vd: " << vd <<  ", v: " << _target_Vy << endl;


//            std::cout<<"numBeacon: "<<numBeacon<<endl;
//            std::cout<<"beaconXloc: "<<beaconXloc<<", beaconYloc: "<<beaconYloc<<endl;
//            std::cout<<"currentX: "<<current_x<<", currentY: "<<current_y<<endl;

//            ii = 0;
                LOOP:
                countTrue = false;
//                std::cout<<"sizeID2detect: "<<ID2detect.size()<<", sizeIDdetected: "<<IDdetected.size()<<", ii: "<<ii<<endl;
//                std::cout<<"xB2detect: "<<xB2detect.size()<<", yB2detect: "<<yB2detect.size()<<", ii: "<<ii<<endl;
                if(ID2detect.size()!=1){
                    if((ii+1)<ID2detect.size()){
                        ii++;
//                        std::cout<<"ii: "<<ii<<endl;
                        curID = ID2detect.at(ii);
                        std::cout<<"1, curID: "<<curID<<endl;
                        if ( std::find(IDdetected.begin(), IDdetected.end(),curID)==IDdetected.end() ){
//                            std::cout<<IDdetected.size()<<endl;
                            IDdetected.push_back(curID);
                            beaconXloc = xB2detect.at(ii);
                            beaconYloc = yB2detect.at(ii);
                            std::cout<<"the: "<<ii<<"th beacon"<<endl;
                            std::cout<<"xB2detect: "<<beaconXloc<<", yB2detect: "<<beaconYloc<<", ii: "<<ii<<endl;
//                          if(abs(current_x-beaconXloc)<diaDetect && abs(current_y-beaconYloc)<diaDetect){
                            curBXloc = beaconXloc;
                            curBYloc = beaconYloc;
                            beaconStartx  = current_x;
                            beaconStarty = current_y;
//                            beaconXlocCount = curBXloc;
//                            beaconYlocCount = curBYloc;
                            sumBXloc = curBXloc;
                            sumBYloc = curBYloc;
                            _wp_index++; // go to 2, straight to small circle
                            count = 1;
//                          }
                        }
                    }
                }


            omegad = _target_Vd/dend;
            thetad = thetad + omegad*dtd;
            omega = _target_V/den;
            theta = theta + omega*dt;

            if(abs(theta-5.3*M_PI)<0.1){
                _wp_index++; // 2
                _wp_index++; // 3
                _wp_index++; // 4
                _wp_index++; // go to stage 5, return home
            }
//            std::cout << "xd: " << xd << ", x: " << current_x <<  ", yd: " << yd <<  ", y: " <<  current_y << endl;
        }



        if(_wp_index == 2){
            denBd = sqrt(pow((curBXloc- current_x),2)+pow((curBYloc - current_y),2));
            _target_Vx = v2SmallCircle*(curBXloc-current_x)/denBd;
            _target_Vy = v2SmallCircle*(curBYloc-current_y)/denBd;

            _target_Vz=0;
            if(abs(current_alt-_target_alt)>tol){
                _target_Vz = -(current_alt-_target_alt)/Tcor;
            }
//            std::cout<<"curBXloc: "<<curBXloc<<", xcur: "<<current_x<<", curBYloc: "<<curBYloc<<", ycur: "<<current_y<<endl;
            if(abs(current_x-curBXloc)<diaSmallCircle && abs(current_y-curBYloc)<diaSmallCircle){
                xBeacond = current_x;
                yBeacond = current_y;
                _wp_index++; // go to 3, small circle
                thetaBd = 0;
            }
        }




        if((_wp_index == 3)){
                denBd = sqrt(pow((curBXloc- xBeacond),2)+pow((curBYloc - yBeacond),2));
                uBeacond = vSmallCircle *(yBeacond-curBYloc)/denBd ;
                vBeacond = vSmallCircle * (curBXloc- xBeacond)/denBd;
                xBeacond = xBeacond+ uBeacond*dtd;
                yBeacond = yBeacond+ vBeacond*dtd;

                _target_Vx = uBeacond;
                _target_Vy = vBeacond;
                errX = xBeacond - current_x;
                if(abs(errX)>tol){
                   _target_Vx = (uBeacond+2*errX)/Tcor;
                 }
                errY= yBeacond - current_y;
                if(abs(errY)>tol){
                   _target_Vy = (vBeacond+2*errY)/Tcor;
                 }
                _target_Vz=0;
                if(abs(current_alt-_target_alt)>tol){
                    _target_Vz = -(current_alt-_target_alt)/Tcor;
                }
                omegaBd = vSmallCircle/denBd;
                thetaBd = thetaBd + omegaBd*dtd;
//                std::cout<<"numBeacon,inside: "<<numBeacon<<endl;
//                std::cout<<"curBXloc: "<<curBXloc<<", curBYloc: "<<curBYloc<<endl;
//                std::cout<<"xBeacon: "<<xBeacond<<", xcur: "<<current_x<<", yBeacon: "<<yBeacond<<", ycur: "<<current_y<<endl;
//                std::cout<<"uBeacon: "<<uBeacond<<", u: "<<_target_Vx<<", vBeacon: "<<vBeacond<<", v: "<<_target_Vy<<", vin: "<<vinBeacon<<endl;
//                std::cout<<"thetaBd: "<<thetaBd<<endl;
//                std::cout<<"denBd : "<<denBd<<endl;


                if(countTrue){
                    sumBXloc = sumBXloc + beaconXlocCount;
                    sumBYloc = sumBYloc + beaconYlocCount;
                    std::cout<<"2, curID: "<<curID<<endl;
                    std::cout << "beaconXlocCount: "<< beaconXlocCount<<", beaconYlocCount: "<< beaconYlocCount<< ", count: "<<count<<endl;
//                    std::cout << "sumBX: "<< sumBXloc<<", sumBY: "<< sumBYloc<< endl;
                    count++;
                    countTrue = false;
                }


               if(abs(thetaBd - 2*M_PI)<0.1){
                _wp_index++; // go to 4, back to where start small circle
               }
        }



        if(_wp_index ==4){
            denBd = sqrt(pow((beaconStartx- current_x),2)+pow((beaconStarty - current_y),2));
            _target_Vx = v2SmallCircle*(beaconStartx-current_x)/denBd;
            _target_Vy = v2SmallCircle*(beaconStarty-current_y)/denBd;

            _target_Vz=0;
            if(abs(current_alt-_target_alt)>tol){
                _target_Vz = -(current_alt-_target_alt)/Tcor;
            }
//            std::cout<<"beaconStartX: "<<beaconStartx<<", xcur: "<<current_x<<", beaconStartY: "<<beaconStarty<<", ycur: "<<current_y<<endl;
            if(abs(current_x-beaconStartx)<0.5 && abs(current_y-beaconStarty)<0.5){
                xBeacond = current_x;
                yBeacond = current_y;
                _wp_index--; //3
                _wp_index--; //2
                _wp_index--; //1

                xBFinal.push_back(sumBXloc/count);
                yBFinal.push_back(sumBYloc/count);
                xBcount.push_back(count);
                yBcount.push_back(count);
                for(int i=0; i < xBFinal.size(); i++){
                      std::cout <<"xBFinal: "<<xBFinal.at(i)<<", yBFinal: "<<yBFinal.at(i)<<", count: "<<xBcount.at(i)<<endl;
                }
                goto LOOP;
            }
        }



        if(_wp_index==5){
            _target_x = x0off;
            _target_y = y0off;
            _target_Vx = 10*(_target_x-current_x)/den;
            _target_Vy = 10*(_target_y-current_y)/den;
            _target_Vz=0;
            if(abs(current_alt-_target_alt)>tol){
                _target_Vz = -(current_alt-_target_alt)/Tcor;
            }
//            std::cout << "u: " << _target_Vx << ", v: " << _target_Vy <<endl;
            if (abs(current_x - _target_x) < 1 || abs(current_y - _target_y) < 1) {
                    _wp_index++;
                    _target_x = x0off;
                    _target_y = y0off;
                    _target_alt = -z0off+5;
            }

        }


         if(_wp_index==6){
             _target_Vz = 10*(_target_alt-current_alt)/den;
             _target_Vx=0;
             if(abs(current_x-_target_x)>tol){
                 _target_Vx = -(current_x-_target_x)/Tcor;
             }
             _target_Vy=0;
             if(abs(current_y-_target_y)>tol){
                 _target_Vy = -(current_y-_target_y)/Tcor;
             }
//             std::cout << "x: " << current_x << ", y: " << current_y<< ", z: " << current_alt <<", zd: " << _target_alt <<endl;
//             std::cout << "u: " << _target_Vx << ", v: " << _target_Vy<< ", w: " << _target_Vz <<endl;
             if (abs(current_alt - _target_alt) < 0.1) {
                     _wp_index++;
             }
         }



         if(_wp_index == 7){
             _target_Vx = 0;
             _target_Vy = 0;
             _target_Vz = 0;
         }

//             while (abs(current_alt - _target_alt) > 0.1){
//                 _target_x = current_x - lx;
//                 _target_y = current_y + ly;
//                 _target_alt = current_alt - lz;
//                 _target_Vx = 0.5*(_target_x)/den;
//                 _target_Vy = 0.5*(_target_y)/den;
//                 _target_Vz = 0.5*(_target_alt)/den;
//                 std::cout << "x: " << current_x << ", y: " << current_y<< ", z: " << current_alt <<", zd: " << _target_alt <<endl;
//                 std::cout << "u: " << _target_Vx << ", v: " << _target_Vy<< ", w: " << _target_Vz <<endl;
//             }

// //            _target_x = current_x - lx;
// //            _target_y = current_y + ly;
// //            _target_alt = current_alt - lz;
// //            _target_Vx = 0.5*(_target_x)/den;
// //            _target_Vy = 0.5*(_target_y)/den;
// //            _target_Vz = 0.5*(_target_alt)/den;
// //            if (abs(current_alt - _target_alt) < 0.1) {
// //                    _wp_index++;
// //            }
//         }


//         if(_wp_index == 6){
//                         _target_Vx = 0;
//                         _target_Vy = 0;
//                         _target_Vz = 0;
//         }


}


void ControlNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
        // TODO: use the information from the measurement as desired
        int num = msg->num_measurements;
//        std::cout<<"total beacon reveiced!!!!!!!!!!!!!!!!!!!!!!!!   "<< num<<"  !!!!!!!!!!!!!!!!!!!!!!!! "<<endl;
        if (num == 0){
                return;
        }

        for(int i=0; i<=num-1; i++){
                numBeacon = msg->id[i];
                beaconYlocReadIn = msg->n[i];
                beaconXlocReadIn = msg->e[i];
//                std::cout<<"reading beacon location now hahahhahahahahhahahahahahhahahahhahhahhahaha"<<endl;
//                std::cout<<"numBeacon: "<<num<<endl;
                if(std::find(ID2detect.begin(), ID2detect.end(),numBeacon)==ID2detect.end()){
                    ID2detect.push_back(numBeacon);
                    xB2detect.push_back(beaconXlocReadIn);
                    yB2detect.push_back(beaconYlocReadIn);
                    std::cout<<"ID size: "<<ID2detect.size()<<endl;
                    for(int j=0; j < ID2detect.size(); j++){
                      std::cout << ID2detect.at(j) << endl;
                    }
                }

//                std::cout<<"i,curID: "<<i<<", "<<curID<<endl;
                if(i==curID){
                    beaconXlocCount = beaconXlocReadIn;
                    beaconYlocCount = beaconYlocReadIn;
                    countTrue =true;
                    std::cout<<"curID: "<<curID<<", i: "<<i<<endl;
//                    std::cout<<"beaconXlocCount: "<<beaconXlocCount<<endl;
                }

        }
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
    float lx=_landing_Pos.pose.position.x;
    float ly=_landing_Pos.pose.position.y;
    float lz=_landing_Pos.pose.position.z;
//    std::cout << "distance:" << lz; //test tmr
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
