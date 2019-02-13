clear;clc 
rosinit('http://192.168.1.136:11311')                       % Connect to ROS network
global cmd_pub s0 count

s0 = 0;
count = 0;

sub1 = rossubscriber('/qualisys_odom',@obs_avoidance);    % Create a subscriber for odometry


cmd_pub = rospublisher('cmd_vel','geometry_msgs/Twist');     % Create a publisher for velocity
