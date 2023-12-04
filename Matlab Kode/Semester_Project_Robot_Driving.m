%% Clear Data

clear all; close all; clc

%% setting environment and creating publishers

setenv("ROS_DOMAIN_ID","5")
test_publisher = ros2node("/test_vm_ros", 5);

cmdPub = ros2publisher(test_publisher, "/cmd_vel", "geometry_msgs/Twist");
recievePub = ros2subscriber(test_publisher, "/odom","nav_msgs/Odometry","Reliability","besteffort","Durability","volatile","Depth",5);
trajectoryPub = ros2publisher(test_publisher,"/joint_trajectory","nav_msgs/Path","Reliability","besteffort","Durability","volatile","Depth",5);
recieveSub = ros2subscriber(test_publisher, "/camera/camera_info","","Reliability","besteffort","Durability","volatile","Depth",5);


cmdMsg = ros2message(cmdPub);
cmdMsg.linear.x = 0.2;
cmdMsg.linear.y = 0.0;
cmdMsg.linear.z = 0.0;
cmdMsg.angular.x = 0.0;
cmdMsg.angular.y = 0.0;
cmdMsg.angular.z = 0.3;
send(cmdPub,cmdMsg)


%Lattice_Planner();

% for cnt = 1:2
%     send(cmdPub,cmdMsg)
%     pause(1)
%     disp("test")
% end

reciveMsg = ros2message(recievePub);


while (true)
    
[scanData,status,statustext] = receive(recievePub,10);


robotPosition = scanData.pose.pose.position
robotOrientation = scanData.pose.pose.orientation
robotAngular = scanData.twist.twist.angular


        self.position = scanData.pose.pose.position;

        x = scanData.pose.pose.orientation.x;
        y = scanData.pose.pose.orientation.y;
        z = scanData.pose.pose.orientation.z;
        w = scanData.pose.pose.orientation.w;

        quaternion = [x y z w]
        euler = quat2eul(quaternion,'XYZ')
        angEul = rad2deg(euler)
        %self.yaw = euler[2]

pause(1)

 % Generate lattice-based trajectory based on current state and goal
    latticeTrajectory = generateLatticeTrajectory(currentRobotState, goal);

   % Publish the generated trajectory as a ROS2 message
    send(trajectoryPub, latticeTrajectory);

end
