#include <ros/ros.h>
#include <keyboard/Key.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <sstream>

#define RATE_LOOP 0.01f
#define scale 1
#define xy_scale 500
#define z_scale 10
#define txy_scale 500
#define tz_scale 0.5000
#define kp 2.0f
#define ki 1.0f
#define kd 1.0f

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
#define FOUR_KEY 52

class FTControl{
	public:
		FTControl();

	private:
    	ros::NodeHandle nh;                         	//ROS Node Handle

		ros::Publisher seria_pub;						//Publishes controller string values
		ros::Publisher debug_pub;                   	//Publishes Desired F/T after PID

		ros::Subscriber netft_sub;                		//subscriber for net F/T Sensor
		ros::Subscriber keydown_sub;                	//subscriber for keyboard downstroke
		ros::Subscriber keyup_sub;                  	//subscriber for keyboard upstroke
		ros::Timer timer;                           	//ROS timer object

		int dFx, dFy, dFz;								// keyboard manual change of des force
		int dTx, dTy, dTz;								// keyboard manual change of des torque
		int Fx, Fy, Fz;									// commanded force
		int Tx, Ty, Tz;									// commanded torque
		int Fxd, Fyd, Fzd;								// desired force
		int Txd, Tyd, Tzd;								// desired torque
		double Daq[6];									// F/T feedback array
		double r[6], ep[6], ei[6], ed[6], e_last[6];	// ref, pid error terms, holder for last ep[]
		bool arm, control_mode, time_zero;				// states: arming, toggle PID, zero state

		ros::Time data_time_now;						// loop time for controller calculation
		ros::Time data_time_last;						// loop time last for controller calculation
		double dt;										// change in time from last calc

		geometry_msgs::Wrench U;						// Desired Wrenching torque

    	//callback defines
		void timer_cb(const ros::TimerEvent& event);
		void keydown_cb(const keyboard::KeyConstPtr& keydown);
		void keyup_cb(const keyboard::KeyConstPtr& keyup);
		void netft_cb(const geometry_msgs::WrenchStampedConstPtr& ftDAQ);

    	//function defines
		geometry_msgs::Wrench PID();
};

FTControl::FTControl(){
	timer = nh.createTimer(ros::Duration(RATE_LOOP), &FTControl::timer_cb, this);

	debug_pub = nh.advertise<std_msgs::Float64>("/debug_Float64", 1);
	seria_pub = nh.advertise<std_msgs::String>("/serial_msg", 1);

	keyup_sub = nh.subscribe<keyboard::Key>("/keyboard/keyup", 1, &FTControl::keyup_cb, this);
	keydown_sub = nh.subscribe<keyboard::Key>("/keyboard/keydown", 1, &FTControl::keydown_cb, this);
	netft_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, &FTControl::netft_cb, this);

	data_time_now = ros::Time::now();
	dt = 0.0;

	arm = false;
	control_mode = false;
	time_zero = true;

	dFx = 0;	dFy = 0;	dFz = 0;
	dTx = 0;	dTy = 0;	dTz = 0;
	Fx  = 0;	Fy  = 0;	Fz  = 0;
	Tx  = 0;	Ty  = 0;	Tz  = 0;
	Fxd = 0;	Fyd = 0;	Fzd = 0;
	Txd = 0;	Tyd = 0;	Tzd = 0;

	for(int i = 0; i < 6; i++){
		Daq[i]    = 0.0;
		r[i]      = 0.0;
		ep[i]     = 0.0;
		ei[i]  	  = 0.0;
		ed[i]  	  = 0.0;
		e_last[i] = 0.0;
	}
	/*Daq[0] = 3;
	Daq[1] = -0.1;
	Daq[2] = 11;
	Daq[3] = -0.05;
	Daq[4] = -0.2;
	Daq[5] = -0.1;*/
	r[0] = 2;
	r[1] = 0;
	r[2] = 10;
	r[3] = 0;
	r[4] = 0;
	r[5] = 0;

}

void FTControl::timer_cb(const ros::TimerEvent& event){
	data_time_now = ros::Time::now();
	dt = (data_time_now - data_time_last).toSec();
	data_time_last = data_time_now;

	Fxd = Fxd + scale*dFx;		//r[0] = Fxd;
	Fyd = Fyd + scale*dFy;		//r[1] = Fyd;
	Fzd = Fzd + scale*dFz;		//r[2] = Fzd;
	Txd = Txd + scale*dTx;		//r[3] = Txd;
	Tyd = Tyd + scale*dTx;		//r[4] = Tyd;
	Tzd = Tzd + scale*dTz;		//r[5] = Tzd;

	if(control_mode == true){
		U = FTControl::PID();
		Fx = U.force.x;
		Fy = U.force.y;
		Fz = U.force.z;
		Tx = U.torque.x;
		Ty = U.torque.y;
		Tz = U.torque.z;
	}
	else{
		time_zero = true;
		Fx = Fxd;
		Fy = Fyd;
		Fz = Fzd;
		Tx = Txd;
		Ty = Tyd;
		Tz = Tzd;
	}

	std_msgs::String msg;
    std::ostringstream os;
    //os << '{' << r[0] << ',' << r[1] << ',' << r[2] << ',' << r[3] << ',' << r[4] << ',' << r[5] << '}';
	//os << '{' << Daq[0] << ',' << Daq[1] << ',' << Daq[2] << ',' << Daq[3] << ',' << Daq[4] << ',' << Daq[5] << '}';
	os << '{' << Fx << ',' << Fy << ',' << Fz << ',' << Tx << ',' << Ty << ',' << Tz << '}';
    if(arm == true){
        msg.data = os.str();
    }
	else{
		Fxd = 0;
		Fyd = 0;
		Fzd = 0;
		Txd = 0;
		Tyd = 0;
		Tzd = 0;
		Fx = 0;
		Fy = 0;
		Fz = 0;
		Tx = 0;
		Ty = 0;
		Tz = 0;
		msg.data = "#";
	}
	seria_pub.publish(msg);
}

geometry_msgs::Wrench FTControl::PID(){
	double u[6];

	if(time_zero == true){
		time_zero = false;
		for(int i = 0; i < 6; i++){
			ei[i] = 0.0;
			e_last[i] = Daq[i];
			u[i] = 0.0;
		}
	}
	else{
		//dt = (data_time_now - data_time_last).toSec();
		for(int i = 0; i < 6; i++){
			ep[i] = r[i] - Daq[i];
			ei[i] = ei[i] + ep[i]*dt;
			ed[i] = (ep[i] - e_last[i])/dt;

			e_last[i] = ep[i];
			u[i] = kp*ep[i] + ki*ei[i] + kd*ed[i];
		}

	}
	//data_time_last = data_time_now;

	U.force.x = u[0];
	U.force.y = u[1];
	U.force.z = u[2];
	U.torque.x = u[3];
	U.torque.y = u[4];
	U.torque.z = u[5];
	return U;
}

void FTControl::netft_cb(const geometry_msgs::WrenchStampedConstPtr& ftDAQ){
	data_time_now = ftDAQ -> header.stamp;
	Daq[0] = ftDAQ -> wrench.force.x;
	Daq[1] = ftDAQ -> wrench.force.y;
	Daq[2] = ftDAQ -> wrench.force.z;
	Daq[3] = ftDAQ -> wrench.torque.x;
	Daq[4] = ftDAQ -> wrench.torque.y;
	Daq[5] = ftDAQ -> wrench.torque.z;
}

void FTControl::keydown_cb(const keyboard::KeyConstPtr& keydown){
	int x = keydown -> code;
	switch(x){
		case Q_KEY:
			dFx = 1;
			break;
		case A_KEY:
      		dFx = -1;
			break;
		case W_KEY:
      		dFy = 1;
			break;
		case S_KEY:
      		dFy = -1;
			break;
		case E_KEY:
      		dFz = 1;
			break;
		case D_KEY:
      		dFz = -1;
			break;
		case R_KEY:
      		dTx = 1;
			break;
		case F_KEY:
      		dTx = -1;
			break;
		case T_KEY:
      		dTy = 1;
			break;
		case G_KEY:
      		dTy = -1;
			break;
		case Y_KEY:
			dTz = 1;
			break;
		case H_KEY:
			dTz = -1;
			break;
    	case ONE_KEY:
			arm = true;
      		break;
  		case TWO_KEY:
			arm = false;
      		break;
  		case THREE_KEY:
			control_mode = true;
      		break;
		case FOUR_KEY:
			control_mode = false;
	      	break;
	}
}

void FTControl::keyup_cb(const keyboard::KeyConstPtr& keyup){
	int x = keyup -> code;
  switch(x){
		case Q_KEY:
      		dFx = 0;
      		break;
	    case A_KEY:
	      	dFx = 0;
	      	break;
	    case W_KEY:
	      	dFy = 0;
	      	break;
	   	case S_KEY:
	     	dFy = 0;
	     	break;
	   	case E_KEY:
	     	dFz = 0;
	     	break;
	   	case D_KEY:
	     	dFz = 0;
	     	break;
	   	case R_KEY:
		 	dTx = 0;
	     	break;
	   	case F_KEY:
	     	dTx = 0;
	     	break;
	   	case T_KEY:
	     	dTy = 0;
	     	break;
	   	case G_KEY:
	     	dTy = 0;
	    	break;
	    case Y_KEY:
			dTz = 0;
	      	break;
	    case H_KEY:
			dTz = 0;
	      	break;
    }
}

int main(int argc, char **argv){
	ros::init(argc, argv, "ControlFT");
	ROS_INFO_STREAM("Generate Hexa Desired Control Active!");
	FTControl controlft;

	while(ros::ok()){
		ros::spinOnce();
	}
}
