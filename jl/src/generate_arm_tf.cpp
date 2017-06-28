/*
 * This source file was written to control the manipulator portion of an aerial manipulation platform (Multirotor and coupled
 * 5 degree of freedom manipulator. This code will broadcast transforms of all manipulator frames to the /tf topic in ROS
 * A user can control each servo using the keyboard keys Q,A,W,S,E,D,R,F,Y,H and the user may switch from manual to autonomous
 * tracking modes using 1 and 2 keys. Mode 1 is default and allows manual control, mode 2 is autonomous.
 * Hardware includes an Arbotix-M Controller with AX-12 servos and a usb-webcam.
 *
 * written by: Jameson Lee
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <keyboard/Key.h>
#include <math.h>
#include <jl/jointAngles.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

#define RATE_LOOP 0.007                     //timer callback loop rate
#define BIT_MAX 4096                        //max resolution of servo

#define L1 0.2f                        //Length of Manipulator Link 1
#define L2 0.2f                        //Length of Manipulator Link 2
#define L3 0.18f                        //Length of Manipulator Link 3
#define SERVO0_0 2048
#define SERVO1_0 2048
#define SERVO2_0 2048
#define SERVO3_0 2048
#define SERVO4_0 2048
#define SERVO5_0 2048
#define SERVO_0 2048.0f
#define MANUAL 1
#define AUTOMATIC 2
#define CARTESIAN 3
#define SPEED_SCALE 3
#define CARTESIAN_SPEED_SCALE 0.001
#define ARM_CAMERA_OFFSET 0.1f              //Offset for autonomous calculation (intersect final link with target and this offset)

// ///////////////////////////// //
// Devavit-Hartenberg parameters //
// ///////////////////////////// //

//UAV Frame to Manipulator Static Base
#define alpha0 M_PI
#define a0 0
#define theta1 0
#define d1 0

//Static Base to joint 1 servo (servo0)
#define alpha1 0
#define a1 0
#define d2 0

//joint 1 servo to Link 1 (servo1)
#define alpha2 M_PI/2
#define a2 0
#define d3 0

//Link 1 to Link 2 (servo2)
#define alpha3 0
#define d4 0

//Link 2 to Link 3 (servo3)
#define alpha4 0
#define d5 0

//Link 3 to wrist (servo4)
#define theta6 0
#define d6 0

// //////////////// //
// Keyboard Defines //
// //////////////// //

#define Q_KEY 113
#define A_KEY 97
#define W_KEY 119
#define S_KEY 115
#define E_KEY 101
#define D_KEY 100
#define R_KEY 114
#define F_KEY 102
#define T_KEY 116
#define G_KEY 103
#define Y_KEY 121
#define H_KEY 104
#define ONE_KEY 49
#define TWO_KEY 50
#define THREE_KEY 51

class Arm{
	public:
		Arm();

	private:

        ros::NodeHandle nh;                         //ROS Node Handle

        ros::Subscriber keydown_sub;                //subscriber for keyboard downstroke
        ros::Subscriber keyup_sub;                  //subscriber for keyboard upstroke
        ros::Subscriber QR_sub;                     //Subscriber for /visp_auto_tracker/object_position

        ros::Publisher servo_pub;                   //Publishes servo output (0-1023)
        ros::Publisher ba_pub;                      //servo0 control topic (rad)
        ros::Publisher j1_pub;                      //servo1 control topic (rad)
        ros::Publisher j2_pub;                      //servo2 control topic (rad)
        ros::Publisher j3_pub;                      //servo3 control topic (rad)

        ros::Timer timer;                           //ROS timer object

        tf::TransformBroadcaster br;                //tf broadcaster object
        tf::TransformListener Li;                   //tf Listener object

        tf::Transform TU;                           //Universal Frame Transform
        tf::Transform T01;                          //UAV to Static Base Frame Transform
        tf::Transform T12;                          //Static Base to Joint 1 servo Frame Transform
        tf::Transform T23;                          //joint 1 servo to link 1 Frame Transform
        tf::Transform T34;                          //link 1 to link 2 Frame Transform
        tf::Transform T45;                          //link 2 to link 3 Frame Transform
        tf::Transform T56;                          //link 3 to wrist Frame Transform
				tf::Transform TG;

        tf::Transform TQR;                          //QR code to Camera Frame Transform
        tf::Transform TCAM;                         //UAV to Camera Frame Transform

        tf::StampedTransform T_goal;                //Base to QR Frame Transform

        tf::Quaternion qU;                          //Quaternion associated with Universal Frame Transform
        tf::Quaternion q01;                         //Quaternion associated with UAV to Static Base Frame Transform
        tf::Quaternion q12;                         //Quaternion associated with Static Base to Joint 1 servo Frame Transform
        tf::Quaternion q23;                         //Quaternion associated with joint 1 servo to link 1 Frame Transform
        tf::Quaternion q34;                         //Quaternion associated with link 1 to link 2 Frame Transform
        tf::Quaternion q45;                         //Quaternion associated with link 2 to link 3 Frame Transform
        tf::Quaternion q56;                         //Quaternion associated with link 3 to wrist Frame Transform

        tf::Quaternion qCAM;                        //Quaternion associated with UAV to Camera Frame Transform
				tf::Quaternion qG;

        ::jl::jointAngles rc;                       //custom message holding all servo outputs (0-1023)

        std_msgs::Float64 th2;                      //variable servo0 control message
        std_msgs::Float64 th3;                      //variable servo1 control message
        std_msgs::Float64 th4;                      //variable servo2 control message
        std_msgs::Float64 th5;                      //variable servo3 control message
        std_msgs::Float64 al5;                      //variable servo4 control message

        geometry_msgs::PoseStamped QR_pose;         //message from visp_auto_tracker package
				geometry_msgs::PoseStamped G_pose;

        float theta2;                               //variable servo0 control (rad)
        float theta3;                               //variable servo1 control (rad)
        float theta4;                               //variable servo2 control (rad)
        float theta5;                               //variable servo3 control (rad)
        float alpha5;                               //variable servo4 control (rad)
        float a3;                                   //Link 1 Length
        float a4;                                   //Link 2 Length
        float a5;                                   //Link 3 Length

        float x_goal;
        float y_goal;
        float z_goal;

        int mode;

        //servo incremental
        int ds0;
        int ds1;
        int ds2;
        int ds3;
        int ds4;

        //callback defines
        void keydown_cb(const keyboard::KeyConstPtr& keydown);
		void keyup_cb(const keyboard::KeyConstPtr& keyup);
        void QR_cb(const geometry_msgs::PoseStampedConstPtr& p);
		void timer_cb(const ros::TimerEvent& event);

        //function defines
        float servo2angle(int servo);
        int angle2servo(float angle);
};

Arm::Arm(){
	rc.m_time = ros::Time::now();
    rc.servo0 = SERVO0_0;
    rc.servo1 = SERVO1_0;
    rc.servo2 = SERVO2_0;
    rc.servo3 = SERVO3_0;
    rc.servo4 = SERVO4_0;
    rc.servo5 = SERVO5_0;
    ds0 = 0;
    ds1 = 0;
    ds2 = 0;
    ds3 = 0;
    ds4 = 0;

	a3 = L1;
	a4 = L2;
	a5 = L3;
    timer = nh.createTimer(ros::Duration(RATE_LOOP), &Arm::timer_cb, this);
	keyup_sub = nh.subscribe<keyboard::Key>("/keyboard/keyup", 1, &Arm::keyup_cb, this);
	keydown_sub = nh.subscribe<keyboard::Key>("/keyboard/keydown", 1, &Arm::keydown_cb, this);
    QR_sub = nh.subscribe<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position", 1, &Arm::QR_cb, this);
    ba_pub = nh.advertise<std_msgs::Float64>("/base_joint/command", 1);
    j1_pub = nh.advertise<std_msgs::Float64>("/j1/command", 1);
    j2_pub = nh.advertise<std_msgs::Float64>("/j2/command", 1);
    j3_pub = nh.advertise<std_msgs::Float64>("/j3/command", 1);
    servo_pub = nh.advertise<jl::jointAngles>("/servo_msg", 1);

    QR_pose.pose.position.x = 0;
    QR_pose.pose.position.y = 0;
    QR_pose.pose.position.z = 0;
    QR_pose.pose.orientation.x = 0;
    QR_pose.pose.orientation.y = 0;
    QR_pose.pose.orientation.z = 0;
    QR_pose.pose.orientation.w = 1;

    x_goal = 0.3f;
    y_goal = 0.0f;
    z_goal = -0.2f;
		G_pose.pose.position.x = x_goal;
    G_pose.pose.position.y = y_goal;
    G_pose.pose.position.z = z_goal;
    mode = MANUAL;
}

float Arm::servo2angle(int servo){
    float x = (float(servo) - SERVO_0)*(M_PI/2)/SERVO_0;
    if(x > M_PI/2) return M_PI/2;
    else if(x < -M_PI/2) return -M_PI/2;
    else return x;
}

int Arm::angle2servo(float angle){
    int x = angle*SERVO_0/(M_PI/2) + SERVO_0;
    if(x > BIT_MAX) return BIT_MAX;
    else if(x < 0) return 0;
    else return x;
}

void Arm::timer_cb(const ros::TimerEvent& event){

    int scale = SPEED_SCALE;
    float scale_c = CARTESIAN_SPEED_SCALE;

    if(mode == MANUAL){
        rc.servo0 = rc.servo0 + scale*ds0;
        rc.servo1 = rc.servo1 + scale*ds1;
        rc.servo2 = rc.servo2 + scale*ds2;
        rc.servo3 = rc.servo3 + scale*ds3;
        rc.servo4 = rc.servo4 + scale*ds4;
    }
    else if(mode == AUTOMATIC || mode == CARTESIAN){
        float x_s;
        float y_s;
        float z_s;

        if(mode == AUTOMATIC){
            try{
                Li.lookupTransform("UAV", "QR", ros::Time(0), T_goal);
            }
            catch(tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }
            x_s = T_goal.getOrigin().x();
            y_s = T_goal.getOrigin().y();
            z_s = T_goal.getOrigin().z();
        }
        else{
            x_goal = x_goal + scale_c*ds0;
            y_goal = y_goal + scale_c*ds1;
            z_goal = z_goal + scale_c*ds2;
            x_s = x_goal;
            y_s = y_goal;
            z_s = z_goal;
        }

        float x = x_s;
        float y = y_s;
        float z = z_s + d1;

        float rho = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        float rhoMax = L1 + L2 + 0.9f*L3; /**/
				/*float rhoMax = 0.5; /**/
        float rhoMin = 0.1f*(L1 + L2);

        if(rho > rhoMax){
            x = x*rhoMax/rho;
            y = y*rhoMax/rho;
            z = z*rhoMax/rho;
        }
        else if(rho < rhoMin){
            x = x*rhoMin/rho;
            y = y*rhoMin/rho;
            z = z*rhoMin/rho;
        }

        float r = sqrt(pow(x,2) + pow(y,2));
        float theta_B =  -atan2(y, x);
        float theta_s = atan2(z + ARM_CAMERA_OFFSET, r);

        float a = pow(L3,2) + pow(r,2) + pow(z,2) - 2*L3*(r*cos(theta_s) + z*sin(theta_s));
        float check = 4*pow(L1,2)*a - pow(pow(L2,2) - pow(L1,2) - a,2);

        float y11 = 2*L1*(-z + L3*sin(theta_s)) + sqrt(4*pow(L1,2)*a - pow(pow(L2,2) - pow(L1,2) - a,2));
        float y12 = 2*L1*(-z + L3*sin(theta_s)) - sqrt(4*pow(L1,2)*a - pow(pow(L2,2) - pow(L1,2) - a,2));
        float x_1 = 2*L1*(-r + L3*cos(theta_s)) + pow(L2,2) - pow(L1,2) - a;

        float y21 = 2*L2*(-z + L3*sin(theta_s)) + sqrt(4*pow(L2,2)*a - pow(pow(L1,2) - pow(L2,2) - a,2));
        float y22 = 2*L2*(-z + L3*sin(theta_s)) - sqrt(4*pow(L2,2)*a - pow(pow(L1,2) - pow(L2,2) - a,2));
        float x_2 = 2*L2*(-r + L3*cos(theta_s)) + pow(L1,2) - pow(L2,2) - a;

        if(check > 0){
            float theta11 = 2*atan2(y11, x_1);
            float theta12 = 2*atan2(y12, x_1);
            float theta21 = 2*atan2(y21, x_2);
            float theta22 = 2*atan2(y22, x_2);

            float ths[4];
            ths[0] = fmod(theta_B, 2.0f*M_PI);
            ths[1] = fmod(theta11, 2.0f*M_PI);
            ths[2] = fmod(-ths[1] + theta22, 2.0f*M_PI);
            ths[3] = fmod(-ths[1] - ths[2] + theta_s, 2.0f*M_PI);


            for(int i = 0; i < 4; i++){
                if(ths[i] > M_PI) ths[i] = ths[i] - 2*M_PI;
                else if(ths[i] < -M_PI) ths[i] = ths[i] + 2*M_PI;
            }

            rc.servo0 = angle2servo(ths[0]);
            rc.servo1 = angle2servo(- M_PI/2 - ths[1] - 0.146);
            rc.servo2 = angle2servo(+ M_PI/2 -ths[2]);
            rc.servo3 = angle2servo(-ths[3]);
        }
    }

    else if(mode == CARTESIAN){


    }

    if(rc.servo0 > BIT_MAX) rc.servo0 = BIT_MAX;
    else if(rc.servo0 < 0) rc.servo0 = 0;
    if(rc.servo1 > BIT_MAX) rc.servo1 = BIT_MAX;
    else if(rc.servo1 < 0) rc.servo1 = 0;
    if(rc.servo2 > BIT_MAX) rc.servo2 = BIT_MAX;
    else if(rc.servo2 < 0) rc.servo2 = 0;
    if(rc.servo3 > BIT_MAX) rc.servo3 = BIT_MAX;
    else if(rc.servo3 < 0) rc.servo3 = 0;
    if(rc.servo4 > BIT_MAX) rc.servo4 = BIT_MAX;
    else if(rc.servo4 < 0) rc.servo4 = 0;

    theta2 = servo2angle(rc.servo0);
    theta3 = servo2angle(rc.servo1);
    theta4 = servo2angle(rc.servo2);
    theta5 = servo2angle(rc.servo3);
    alpha5 = servo2angle(rc.servo4);

    th2.data = theta2;
    th3.data = theta3;
    th4.data = theta4;
    th5.data = theta5;

	q01.setRPY(alpha0,0,theta1);
	q12.setRPY(alpha1,0,theta2);
    q23.setRPY(alpha2,-M_PI/2 - theta3 - 0.146,0);
	q34.setRPY(alpha3,0,-M_PI/2 + theta4);
	q45.setRPY(alpha4,0,theta5);
    q56.setRPY(alpha5,0,theta6);

    qU.setRPY(0,0,0);
    qCAM.setRPY(-M_PI/2,0,-M_PI/2);

	T01.setOrigin(tf::Vector3(a0,-d1*sin(alpha0),d1*cos(alpha0)));
	T01.setRotation(q01);
	T12.setOrigin(tf::Vector3(a1,-d2*sin(alpha1),d2*cos(alpha1)));
	T12.setRotation(q12);
	T23.setOrigin(tf::Vector3(a2,-d3*sin(alpha2),d3*cos(alpha2)));
	T23.setRotation(q23);
	T34.setOrigin(tf::Vector3(a3,-d4*sin(alpha3),d4*cos(alpha3)));
	T34.setRotation(q34);
	T45.setOrigin(tf::Vector3(a4,-d5*sin(alpha4),d5*cos(alpha4)));
	T45.setRotation(q45);
	T56.setOrigin(tf::Vector3(a5,-d6*sin(alpha5),d6*cos(alpha5)));
	T56.setRotation(q56);


    TU.setOrigin(tf::Vector3(1,1,1));
    TU.setRotation(qU);
    TCAM.setOrigin(tf::Vector3(0.10795,0,0.254));
    TCAM.setRotation(qCAM);
    TQR.setOrigin(tf::Vector3(QR_pose.pose.position.x,QR_pose.pose.position.y,QR_pose.pose.position.z));
    TQR.setRotation(tf::Quaternion(QR_pose.pose.orientation.x,QR_pose.pose.orientation.y,QR_pose.pose.orientation.z,QR_pose.pose.orientation.w));
		TG.setOrigin(tf::Vector3(x_goal,y_goal,z_goal));
		TG.setRotation(tf::Quaternion(0,0,0,1));

	rc.m_time = ros::Time::now();

  br.sendTransform(tf::StampedTransform(TU,rc.m_time,"UNIVERSAL","UAV"));
	br.sendTransform(tf::StampedTransform(TG,rc.m_time,"UAV","GOAL"));
	br.sendTransform(tf::StampedTransform(T01,rc.m_time,"UAV","Base"));
	br.sendTransform(tf::StampedTransform(T12,rc.m_time,"Base","joint1"));
	br.sendTransform(tf::StampedTransform(T23,rc.m_time,"joint1","joint2"));
	br.sendTransform(tf::StampedTransform(T34,rc.m_time,"joint2","joint3"));
	br.sendTransform(tf::StampedTransform(T45,rc.m_time,"joint3","wrist"));
	br.sendTransform(tf::StampedTransform(T56,rc.m_time,"wrist","gripper"));

    br.sendTransform(tf::StampedTransform(TCAM,rc.m_time,"UAV","CAM"));
    br.sendTransform(tf::StampedTransform(TQR,rc.m_time,"CAM","QR"));

    servo_pub.publish(rc);
    ba_pub.publish(th2);
    j1_pub.publish(th3);
    j2_pub.publish(th4);
    j3_pub.publish(th5);
}

void Arm::QR_cb(const geometry_msgs::PoseStampedConstPtr& p){
    QR_pose.pose = p -> pose;
}

void Arm::keydown_cb(const keyboard::KeyConstPtr& keydown){
	int x = keydown -> code;
	switch(x){
		case Q_KEY:
            ds0 = 1;
			break;
		case A_KEY:
            ds0 = -1;
			break;
		case W_KEY:
            ds1 = 1;
			break;
		case S_KEY:
            ds1 = -1;
			break;
		case E_KEY:
            ds2 = 1;
			break;
		case D_KEY:
            ds2 = -1;
			break;
		case R_KEY:
            ds3 = 1;
			break;
		case F_KEY:
            ds3 = -1;
			break;
		case T_KEY:
            ds4 = 1;
			break;
		case G_KEY:
            ds4 = -1;
			break;
		case Y_KEY:

			break;
		case H_KEY:

			break;
        case ONE_KEY:
            mode = MANUAL;
            break;
        case TWO_KEY:
            mode = AUTOMATIC;
            break;
        case THREE_KEY:
            mode = CARTESIAN;
            break;
	}
}

void Arm::keyup_cb(const keyboard::KeyConstPtr& keyup){
    int x = keyup -> code;
    switch(x){
        case Q_KEY:
            ds0 = 0;
            break;
        case A_KEY:
            ds0 = 0;
            break;
        case W_KEY:
            ds1 = 0;
            break;
        case S_KEY:
            ds1 = 0;
            break;
        case E_KEY:
            ds2 = 0;
            break;
        case D_KEY:
            ds2 = 0;
            break;
        case R_KEY:
            ds3 = 0;
            break;
        case F_KEY:
            ds3 = 0;
            break;
        case T_KEY:
            ds4 = 0;
            break;
        case G_KEY:
            ds4 = 0;
            break;
        case Y_KEY:

            break;
        case H_KEY:

            break;
    }
}

int main(int argc, char **argv){
	ros::init(argc, argv, "Arm");
	ROS_INFO_STREAM("Generate Arm tf Node Active!");
	Arm arm;

	while(ros::ok()){
		ros::spinOnce();
	}
}
