/**
 * skeleton / example code for a node to do command and control of the pixhawk
 */

// includes
#include <math.h>
#include <ros/ros.h>

// topic data
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/PersonEstimate.h>
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


        //           PAUL VARIABLES //
        float PAUL_target_U_lake = 20.0f; // This is the altitude desired w/r to lagunita frame
        float PAUL_target_E_lake=0.0f;
        float PAUL_target_N_lake=0.0f;
        float PAUL_target_v_E=0.0f;
        float PAUL_target_v_N=0.0f;
        float PAUL_target_v_U=0.0f;
        float PAUL_target_yaw=0.0f; // Pointing east by default


        float PAUL_target_U_lake_c = 20.0f; // This is the altitude desired w/r to lagunita frame
        float PAUL_target_E_lake_c=0.0f;
        float PAUL_target_N_lake_c=0.0f;

        geometry_msgs::TwistStamped PAUL_current_local_speed;

        // Gains of the PD
        float PAUL_Kp_x=0.1f;
        float PAUL_Kp_y=0.1f;
        float PAUL_Kp_z=0.5f;
        float PAUL_Kd_x=0.1f;
        float PAUL_Kd_y=0.1f;
        float PAUL_Kd_z=0.5f;


        // Landing position
        float PAUL_landing_e_lake=0.0f;
        float PAUL_landing_n_lake=0.0f;

        // CAMERA ACQUISITION
        int PAUL_wp_index_start_camera=-5;
        float PAUL_lx=0.0f; // POITING SOUTH
        float PAUL_ly=0.0f; // POITING WEST
        float PAUL_lz=0.0f; //true up altidude


        // Maximum speed control for drone
        float PAUL_u_speed_max=5.0f;
        float PAUL_v_z_speed_max=3.0f;


        // DEFINE PI
        double PAUL_pi = 3.1415926535897;

        // END PAUL VARIABLES //




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

        int _wp_index = -2;
        int _n_waypoints = 1;
        float _target_alt = 0.0f;
        float _target_Vd = 4.0f;
        float _target_V = 4.0f;
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
        float dtd = 0.1f;
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
        float lx = 0.0f; // CAMERA INFORMATION DOWN SOUTH
        float ly = 0.0f; // WEST
        float lz = 0.0f; // DOWN
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
        double vSmallCircle = 1.5f;
        double v2SmallCircle = 4.0f;
        float sumBXloc= 0.0f;
        float sumBYloc = 0.0f;
        float diaSmallCircle = 8.0f;
        bool countTrue = false;
        float beaconXlocCount= 0.0f;
        float beaconYlocCount = 0.0f;
        float totalCircle = 3.0f;

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


        //            PAUL              ///


        ros::Subscriber PAUL_local_speed_sub;		// local velocity information

        // END PAUL //

        // publishers
        ros::Publisher _cmd_pub;
        ros::Publisher _person_pub;
ros::Publisher _wp_pub;
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



        //             PAUL //
        /**
         * callback for the local speed computed by the pixhawk.
        * @param msg pose stamped message type
         */
        void localSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

        // END PAUL //

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
        _sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 15, &ControlNode::sensorMeasCallback, this);
        _mission_state_sub = _nh.subscribe<aa241x_mission::MissionState>("mission_state", 10, &ControlNode::missionStateCallback, this);
_landing_pose_sub =  _nh.subscribe<geometry_msgs::PoseStamped>("landing_pose", 1, &ControlNode::landingPoseCallback, this);

        //    PAUL  //
        PAUL_local_speed_sub = _nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, &ControlNode::localSpeedCallback, this);

        // END PAUL //
        // advertise the published detailed

        // publish a PositionTarget to the `/mavros/setpoint_raw/local` topic which
        // mavros subscribes to in order to send commands to the pixhawk
        _cmd_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
        _person_pub = _nh.advertise<aa241x_mission::PersonEstimate>("person_found", 10);
_wp_pub = _nh.advertise<aa241x_mission::MissionState>("waypoint", 1);
}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _current_state = *msg;
}


// PAUL //
void ControlNode::localSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        PAUL_current_local_speed=*msg;
}

// END PAUL//



void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // save the current local position locally to be used in the main loop
        // TODO: account for offset to convert from PX4 coordinate to lake lag frame
        _current_local_pos = *msg;
        x0off = _e_offset;
        y0off = _n_offset;
        z0off = _u_offset;


        //     PAUL    //

        float PAUL_current_local_pos_E=_current_local_pos.pose.position.x;
        float PAUL_current_local_pos_N=_current_local_pos.pose.position.y;
        float PAUL_current_local_pos_U=_current_local_pos.pose.position.z;

        // TODO: make sure to account for the offset if desiring to fly in the Lake Lag frame
        // I assume e,n, u offset are component of the vector drone to lake
        float PAUL_current_local_pos_E_lake=_e_offset+PAUL_current_local_pos_E;
        float PAUL_current_local_pos_N_lake=_n_offset+PAUL_current_local_pos_N;
        float PAUL_current_local_pos_U_lake=_u_offset+PAUL_current_local_pos_U;


        // END PAUL //


        float current_alt = _current_local_pos.pose.position.z;
        float current_x = _current_local_pos.pose.position.x;
        float current_y = _current_local_pos.pose.position.y;
        current_x += x0off;
        current_y += y0off;
        current_alt += z0off;
        //float den = sqrt(pow((current_x-x0),2)+pow((current_y-y0),2));
        float den = sqrt(pow((current_x),2)+pow((current_y),2));
	std::cout << "-wp: " << _wp_index <<endl;




        // PAUL first way //
        if (_wp_index == -1) {
                //_target_U_lake=_u_offset+_flight_alt;

                // check condition on being "close enough" to the waypoint
                if (abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5 ) {
                        //ROS_INFO("PASSE AU MODE 1");
                        //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                        // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                        _wp_index++;
                }
        }

        // END PAUL //

               if(_wp_index == 0){
                _target_alt = 30;
                diaDetect = 5/7*_target_alt+28.75-5;
                _target_x = 0;
                _target_y = diaDetect/2;
                _target_Vx = 10*(_target_x-current_x)/den;
                _target_Vy = 10*(_target_y-current_y)/den;

                _target_Vz=0;
                if(abs(current_alt-_target_alt)>tol){
                    _target_Vz = -(current_alt-_target_alt)/Tcor;
                }

                // check condition on being "close enough" to the waypoint
                if (abs(current_x - _target_x) < 1 || abs(current_y - _target_y) < 1) {
                        // update the target altitude to land, and increment the waypoint
                        _wp_index++; // ZIXI TO TRY PAUL SCRIPT NEED TO CHANGE THAT TO ++
                        ID2detect.push_back(100);
                        IDdetected.push_back(100);
                        xB2detect.push_back(100);
                        yB2detect.push_back(100);
                        xd = current_x;
                        yd = current_y;
                        dend0 = sqrt(pow(xd,2)+pow(yd,2));
                        den0 = dend0;
//                        std::cout<<"1, _wp: "<<_wp_index<<endl;
                }
               }


        if (_wp_index == 1) {
            curSpiral = floor(thetad/2/M_PI)+1;
            if(thetad<(curSpiral*2*M_PI - 0.1)){
                    _target_Vind = 5/2/M_PI*log((dend0+diaDetect)/dend0)*1.2;
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
               if((_target_Vy)>=6){
                   _target_Vy = 6;
               }
               if((_target_Vy)<=-6){
                   _target_Vy = -6;
               }
             }
            _target_Vz=0;
            if(abs(current_alt-_target_alt)>tol){
                _target_Vz = -(current_alt-_target_alt)/Tcor;
            }

           // std::cout << "ud: " << ud <<  ", u: " << _target_Vx<< "vd: " << vd <<  ", v: " << _target_Vy<< endl;
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

            if(abs(theta-totalCircle*M_PI)<0.1){
                _wp_index++; // 2
                _wp_index++; // 3
                _wp_index++; // 4
                _wp_index++; // go to stage 5, return home
            }
//            std::cout << "xd: " << xd << ", x: " << current_x <<  ",stanfor yd: " << yd <<  ", y: " <<  current_y << endl;
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
                      std::cout <<"ID: "<<ID2detect.at(i)<<", xBFinal: "<<xBFinal.at(i)<<", yBFinal: "<<yBFinal.at(i)<<", count: "<<xBcount.at(i)<<endl;
                }
                goto LOOP;
            }
        }


        // PAUL //
        // GO TO ABOVE THE LANDING SITE DEFINING THE WAYPOINT COORDINATES
        if(_wp_index==5){
            // DON'T delete, this is the publish function for beacon for score
            //aa241x_mission::PersonEstimate meas;
            //meas.header.stamp = ros::Time::now();

            //for (int i = 0; i < xBFinal.size(); i++) {
                    // add to the message
             //       meas.id=IDdetected.at(i+1);
             //       meas.n=yBFinal.at(i);
             //       meas.e=xBFinal.at(i);
             //       _person_pub.publish(meas);
            //}
//            ROS_INFO("MODE 5");

                            _wp_index++;
                            PAUL_target_E_lake=PAUL_landing_e_lake;
                            PAUL_target_N_lake=PAUL_landing_n_lake;
                            PAUL_target_U_lake=_u_offset+20; // HOW DO I KNOW ABOUT THE TARGET ALTITUDE
                            PAUL_Kp_x=0.5f;
                            PAUL_Kp_y=0.5f;
                            PAUL_Kp_z=0.5f;
                            PAUL_Kd_x=0.5f;
                            PAUL_Kd_y=0.5f;
                            PAUL_Kd_z=0.5f;




            //------------------------------------------------------------




        }


        // GO DOWN BY 10 m
        if (_wp_index == 6) {
//std::cout<<"Elake: "<<PAUL_target_E_lake<<", Nlake: "<<PAUL_target_N_lake<<endl;

//                ROS_INFO("MODE 6");
                // check condition on being "close enough" to the waypoint
                if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5) ) {


                       PAUL_Kp_x=0.1f;
                       PAUL_Kp_y=0.1f;
                       PAUL_Kp_z=0.1f;
                       PAUL_Kd_x=0.1f;
                       PAUL_Kd_y=0.1f;
                       PAUL_Kd_z=0.1f;
                        // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                        _wp_index++;
                        PAUL_target_E_lake=PAUL_landing_e_lake;
                        PAUL_target_N_lake=PAUL_landing_n_lake;
                        PAUL_target_U_lake=_u_offset+4;
                }
        }




         if (_wp_index == 7) {

             aa241x_mission::MissionState results;
             results.header.stamp = ros::Time::now();

             results.mission_state = _wp_index;
             _wp_pub.publish(results);
                 //_target_U_lake=_u_offset+_flight_alt;
             if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                 _wp_index=666;
                 //PAUL_target_E_lake=PAUL_target_E_lake_c;
                 //PAUL_target_N_lake=PAUL_target_N_lake_c;
                 //PAUL_target_U_lake=PAUL_target_U_lake_c;
             }
                 // check condition on being "close enough" to the waypoint
                 if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                         //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                         // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                         _wp_index++;
                         PAUL_target_E_lake=PAUL_landing_e_lake+3;
                         PAUL_target_N_lake=PAUL_landing_n_lake+0;
                         PAUL_Kp_x=0.3f;
                         PAUL_Kp_y=0.3f;
                         PAUL_Kp_z=0.1f;
                         PAUL_Kd_x=0.3f;
                         PAUL_Kd_y=0.3f;
                         PAUL_Kd_z=0.1f;

                 }
         }






         if (_wp_index == 8) {

                 if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake-3;
                          PAUL_target_N_lake=PAUL_landing_n_lake-3;
                  }

         }

         if (_wp_index == 9) {

                 if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake+0;
                          PAUL_target_N_lake=PAUL_landing_n_lake+3;
                  }

         }

         if (_wp_index == 10) {

                  if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake+3;
                          PAUL_target_N_lake=PAUL_landing_n_lake-3;
                  }

         }


         if (_wp_index == 11) {

                 if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake-3;
                          PAUL_target_N_lake=PAUL_landing_n_lake+0;
                  }

         }

         if (_wp_index == 12) {

                  if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20){
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake+3;
                          PAUL_target_N_lake=PAUL_landing_n_lake+3;
                  }

         }

         if (_wp_index == 13) {

                  if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake+0;
                          PAUL_target_N_lake=PAUL_landing_n_lake-3;
                  }

         }

         if (_wp_index == 14) {

                  if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake-3;
                          PAUL_target_N_lake=PAUL_landing_n_lake+3;
                  }
         }


         if (_wp_index == 15) {

                 if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20) {
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                          PAUL_target_E_lake=PAUL_landing_e_lake+0;
                          PAUL_target_N_lake=PAUL_landing_n_lake+0;
                  }
         }


         if (_wp_index == 16) {

                  if ((PAUL_lx!=0 || PAUL_ly!=0 || PAUL_lz!=0) && abs(PAUL_lz)<20){
                        _wp_index=666;
                        //PAUL_target_E_lake=PAUL_target_E_lake_c;
                        //PAUL_target_N_lake=PAUL_target_N_lake_c;
                        //PAUL_target_U_lake=PAUL_target_U_lake_c;
                  }
                  if ((abs(PAUL_current_local_pos_U_lake - PAUL_target_U_lake) < 0.5) && (abs(PAUL_current_local_pos_N_lake - PAUL_target_N_lake)< 0.5) && (abs(PAUL_current_local_pos_E_lake - PAUL_target_E_lake)< 0.5)) {

                          //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                          // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                          _wp_index++;
                  }
         }




         if (_wp_index == 666) {
                 //_target_U_lake=_u_offset+_flight_alt;
                PAUL_Kp_x=0.1f;
                PAUL_Kp_y=0.1f;
                PAUL_Kp_z=0.1f;
                PAUL_Kd_x=0.05f;
                PAUL_Kd_y=0.05f;
                PAUL_Kd_z=0.1f;
                 // check condition on being "close enough" to the waypoint
                 if (abs(PAUL_lz)<0.75) {

                         //ROS_INFO("valeur de Lake to Drone: %f",_current_local_pos_U_lake);
                         // increment the waypoint to go to the second waypoint "Computing the value of crossing point with line"
                         _wp_index++;


                 }
         }





         // END PAUL //



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
//                    std::cout<<"ID size: "<<ID2detect.size()<<endl;
//                    for(int j=0; j < ID2detect.size(); j++){
//                      std::cout << ID2detect.at(j) << endl;
//                    }
                }

//                std::cout<<"i,curID: "<<i<<", "<<curID<<endl;
                if(i==curID){
                    beaconXlocCount = beaconXlocReadIn;
                    beaconYlocCount = beaconYlocReadIn;
                    countTrue =true;
//                    std::cout<<"curID: "<<curID<<", i: "<<i<<endl;
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
        PAUL_landing_e_lake=msg->landing_e;
        PAUL_landing_n_lake=msg->landing_n;
        //std::cout << "offset" <<_e_offset << "state"<<state;
}


void  ControlNode::landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    _landing_Pos = *msg;
    float lx=_landing_Pos.pose.position.x;
    float ly=_landing_Pos.pose.position.y;
    float lz=_landing_Pos.pose.position.z;
//    std::cout << "distance:" << lz; //test tmr

    if (_wp_index >= 7) { // TO BE CHANGED
        PAUL_lx=_landing_Pos.pose.position.x;
        PAUL_ly=_landing_Pos.pose.position.y;
        PAUL_lz=_landing_Pos.pose.position.z;
    // Update the target
        float PAUL_current_local_pos_E_c=_current_local_pos.pose.position.x;
        float PAUL_current_local_pos_N_c=_current_local_pos.pose.position.y;
        float PAUL_current_local_pos_U_c=_current_local_pos.pose.position.z;
        float PAUL_current_local_pos_E_lake_c=_e_offset+PAUL_current_local_pos_E_c;
        float PAUL_current_local_pos_N_lake_c=_n_offset+PAUL_current_local_pos_N_c;
        float PAUL_current_local_pos_U_lake_c=_u_offset+PAUL_current_local_pos_U_c;

        PAUL_target_E_lake_c=PAUL_current_local_pos_E_lake_c-PAUL_ly;
        PAUL_target_N_lake_c=PAUL_current_local_pos_N_lake_c-PAUL_lx;
        PAUL_target_U_lake_c=PAUL_current_local_pos_U_lake_c-PAUL_lz;
    }


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
        cmd.yaw = 0;    // PAUL MODFICATION


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



        // PAUL //

        // the position information for the command
        // NOTE: this is defined in ENU
        geometry_msgs::Point PAUL_pos;
        PAUL_pos.x = 0;	// E
        PAUL_pos.y = 0;	// N
        PAUL_pos.z = 0;	// U

        // the velocity information for the command
        // NOTE: this is defined in ENU
        geometry_msgs::Vector3 PAUL_vel;

        PAUL_vel.x = 0;	// E
        PAUL_vel.y = 0;	// N
        PAUL_vel.z = 0;	// U


        // END PAUL //


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

                // set the first waypoint

                // TODO: populate the control elements desired
                //
                // in this case, just asking the pixhawk to takeoff to the _target_alt
                // height
                if((_target_Vx)>=5){
                    _target_Vx = 5;
                }
                if((_target_Vx)<=-5){
                    _target_Vx = -5;
                }
                if((_target_Vy)>=5){
                    _target_Vy = 5;
                }
                if((_target_Vy)<=-5){
                    _target_Vy = -5;
                }
                if((_target_Vz)>=5){
                    _target_Vz = 5;
                }
                if((_target_Vz)<=-5){
                    _target_Vz = -5;
                }
                vel.x = _target_Vx;
                vel.y = _target_Vy;
                vel.z = _target_Vz;
//                pos.z = _target_alt;
//                std::cout << "velX: " << vel.x << ",velY: " << vel.y <<  ", velZ: " << vel.z  <<endl;



                //                           PAUL                    //


                // set the first waypoint, we want a mimic takeoff 15m above the current pos
                if (_wp_index == -2) {
                        PAUL_target_N_lake=_current_local_pos.pose.position.y+_n_offset;
                        PAUL_target_E_lake=_current_local_pos.pose.position.x+_e_offset;
                        PAUL_target_U_lake=_current_local_pos.pose.position.z+_u_offset+20;
                        _wp_index=-1;
                }
                // DEFINITION OF THE PD CONTROLLER




                float PAUL_current_local_pos_E=_current_local_pos.pose.position.x;
                float PAUL_current_local_pos_N=_current_local_pos.pose.position.y;
                float PAUL_current_local_pos_U=_current_local_pos.pose.position.z;

                // Warning: we control everything in the local frame: need to define a control in the drone initial frame
                PAUL_pos.x = PAUL_target_E_lake-_e_offset;
                PAUL_pos.y = PAUL_target_N_lake-_n_offset;
                PAUL_pos.z = PAUL_target_U_lake-_u_offset;


                float PAUL_current_local_speed_E=PAUL_current_local_speed.twist.linear.x;
                float PAUL_current_local_speed_N=PAUL_current_local_speed.twist.linear.y;
                float PAUL_current_local_speed_U=PAUL_current_local_speed.twist.linear.z;


                // We compute the error in the x and y directions in the local frame (same as lake by the way)
                float PAUL_epsilon_x=-PAUL_current_local_pos_E+PAUL_pos.x;
                float PAUL_epsilon_y=-PAUL_current_local_pos_N+PAUL_pos.y;
                float PAUL_epsilon_z=-PAUL_current_local_pos_U+PAUL_pos.z;

//                std::cout<<"Epsilon error altitude: "<<PAUL_epsilon_z<<endl;
                float PAUL_epsilon_speed_x=PAUL_current_local_speed_E;
                if(PAUL_epsilon_x>0 && PAUL_current_local_speed_E>0) {
                    PAUL_epsilon_speed_x=PAUL_current_local_speed_E;
                } else if(PAUL_epsilon_x>0 && PAUL_current_local_speed_E<0) {
                    PAUL_epsilon_speed_x=-PAUL_current_local_speed_E;
                } else if(PAUL_epsilon_x<0 && PAUL_current_local_speed_E<0) {
                    PAUL_epsilon_speed_x=PAUL_current_local_speed_E;
                } else {
                    PAUL_epsilon_speed_x=-PAUL_current_local_speed_E;
                }
                float PAUL_epsilon_speed_y=PAUL_current_local_speed_N;
                if(PAUL_epsilon_y>0 && PAUL_current_local_speed_N>0) {
                    PAUL_epsilon_speed_y=PAUL_current_local_speed_N;
                } else if(PAUL_epsilon_y>0 && PAUL_current_local_speed_N<0) {
                    PAUL_epsilon_speed_y=-PAUL_current_local_speed_N;
                } else if(PAUL_epsilon_y<0 && PAUL_current_local_speed_N<0) {
                    PAUL_epsilon_speed_y=PAUL_current_local_speed_N;
                } else {
                    PAUL_epsilon_speed_y=-PAUL_current_local_speed_N;
                }

                float PAUL_epsilon_speed_z=PAUL_current_local_speed_U;
                if(PAUL_epsilon_z>0 && PAUL_current_local_speed_U>0) {
                    PAUL_epsilon_speed_z=PAUL_current_local_speed_U;
                } else if(PAUL_epsilon_z>0 && PAUL_current_local_speed_U<0) {
                    PAUL_epsilon_speed_z=-PAUL_current_local_speed_U;
                } else if(PAUL_epsilon_z<0 && PAUL_current_local_speed_U<0) {
                    PAUL_epsilon_speed_z=PAUL_current_local_speed_U;
                } else {
                    PAUL_epsilon_speed_z=-PAUL_current_local_speed_U;
                }
//                std::cout<<"Epsilon error altitude after modification: "<<PAUL_epsilon_z<<endl;

                //compute the desired yaw to always face the target position
                PAUL_target_yaw=atan(PAUL_epsilon_y/PAUL_epsilon_x);
                if (PAUL_current_local_pos_E>PAUL_pos.x) {
                    PAUL_target_yaw+=PAUL_pi;
                }

                // WE RELY ON THE CAMERA
                if (_wp_index == 666) {
                    PAUL_epsilon_x=-PAUL_ly;
                    PAUL_epsilon_y=-PAUL_lx;
                    PAUL_epsilon_z=-PAUL_lz;
    //                PAUL_target_E_lake=PAUL_target_E_lake_c;
     //               PAUL_target_N_lake=PAUL_target_N_lake_c;
      //              PAUL_target_U_lake=PAUL_target_U_lake_c;
                }

                PAUL_vel.x=PAUL_Kp_x*PAUL_epsilon_x + PAUL_Kd_x*PAUL_epsilon_speed_x;
                PAUL_vel.y=PAUL_Kp_y*PAUL_epsilon_y + PAUL_Kd_y*PAUL_epsilon_speed_y;
                PAUL_vel.z=PAUL_Kp_z*PAUL_epsilon_z + PAUL_Kd_z*PAUL_epsilon_speed_z;
//                std::cout<<"Velocity required: "<<PAUL_vel.z<<endl;

                if(abs(pow(PAUL_vel.x,2)+pow(PAUL_vel.y,2))>PAUL_u_speed_max) {
                    PAUL_vel.x=PAUL_u_speed_max*cos(PAUL_target_yaw);
                    PAUL_vel.y=PAUL_u_speed_max*sin(PAUL_target_yaw);
                }

                if(abs(PAUL_vel.z)>PAUL_v_z_speed_max) {
                    PAUL_vel.z=PAUL_vel.z/abs(PAUL_vel.z)*PAUL_v_z_speed_max;
                }


                if (_wp_index ==-1) {
                    vel.z=PAUL_vel.z;
                    vel.x=PAUL_vel.x;
                    vel.y=PAUL_vel.y;
                    PAUL_target_yaw=PAUL_pi/2;
                    cmd.yaw=PAUL_pi/2;
                }

                if(_wp_index==666 ||_wp_index == 6 || _wp_index ==7 || _wp_index ==5 || _wp_index == 8 || _wp_index == 9 || _wp_index == 10 || _wp_index == 11 || _wp_index == 12 || _wp_index == 13 || _wp_index == 14 || _wp_index == 15 || _wp_index == 16|| _wp_index == 17) {

                    vel.x=PAUL_vel.x;
                    vel.y=PAUL_vel.y;
                    vel.z=PAUL_vel.z;
//                    std::cout<<"UPDATE ON VEL.z: "<<vel.z<<endl;
                }
//                std::cout<<"UPDATE ON VEL.z: "<<vel.z<<endl;


                if(_wp_index==17) {
                    vel.z=-0.2;
                    vel.x=0;
                    vel.y=0;
                    PAUL_target_yaw=PAUL_pi/2;
                }


                if(_wp_index == 666 & abs(vel.z)>0.2) { //CONDTION TO MAKE SURE THE DRONE DON'T GO TO FAST WHEN WE RELY ON CAMERA
                    vel.z=-0.2;
                }

                if(_wp_index==667) {
                    vel.z=-0.1;
                    vel.x=0;
                    vel.y=0;
                    PAUL_target_yaw=PAUL_pi/2;
                }

                // END PAUL //




                // publish the command
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
