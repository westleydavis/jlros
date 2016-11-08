#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/approximate_time.h>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_type;      

class Sync{
	public:
		Sync();
	private:
		ros::NodeHandle nh;
		ros::Publisher sync_image_pub;
  		ros::Publisher sync_info_pub;
		message_filters::Subscriber<sensor_msgs::Image> *image_sub;
		message_filters::Subscriber<sensor_msgs::CameraInfo> *info_sub;
		message_filters::Synchronizer<MySyncPolicy> *sync;

		void sync_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
};
	
Sync::Sync(){
	sync_image_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/image_mono_sync", 1);
	sync_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/usb_cam/camera_info_sync", 1);

	image_sub = new image_sub_type(nh, "/usb_cam/image_mono", 1);
	info_sub = new info_sub_type(nh, "/usb_cam/camera_info", 1);
  
  	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *info_sub);
  	sync -> registerCallback(boost::bind(&Sync::sync_cb, this, _1, _2));
}

void Sync::sync_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){
	
	sync_image_pub.publish(image);
	sync_info_pub.publish(cam_info);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "Sync");
	ROS_INFO_STREAM("Sync Camera Node Active!");
	Sync sync;

	while(ros::ok()){
		ros::spin();
	}
}


