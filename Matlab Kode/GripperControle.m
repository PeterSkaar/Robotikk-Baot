

clc; 
clear; 
close all;
import ETS3.*

%% Ros node
% Setting up environment
setenv("ROS_DOMAIN_ID","5");

armControllerNode = ros2node("/open_manipulator_controller");


%% Ros Message
Gripper = ros2publisher(armControllerNode, "/gripper_control/commands", "std_msgs/Float64MultiArray");
Effort = ros2publisher(armControllerNode, "/gripper_control/commands","std_msgs/")

Grippermessage = ros2message(Gripper);

Grippermessage.data = 0.044;

send(Gripper,Grippermessage);


%% Close Grippper

prevalue = 0.00;
for cnt = 1:8
    pause(2)

Grippermessage.data = prevalue;
addvalue = 0.01;
prevalue = addvalue + prevalue;
send(Gripper,Grippermessage);
disp("message sent")
end

%% Open Gripper

if (prevalue >= 0.07)
for cnt = 1:8
    pause(2)

Grippermessage.data = prevalue;
addvalue = -0.01;
prevalue = addvalue + prevalue;
send(Gripper,Grippermessage);
disp("message sent")
end
end
  
