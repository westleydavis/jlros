# jlros
custom ROS package for school

current external packages required for full project:

arbotix_ros

image_pipeline

ros-keyboard

usb_cam

vision_visp

Please add the folder "models" to your corresponding vision_visp/visp_auto_tracker/ (you should have a models folder present with generic configuration, this is current QR we are using at our lab)

Please add "arm.yaml" configuration file to "arbotix_ros/arbotix_python/ (This should let your arbotix controller know what topics to listen to, publish rates for servos, etc...)

