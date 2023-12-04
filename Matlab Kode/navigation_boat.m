%% ELE306 Navigation Strategy Lattis Planner BOAT
clc; 
clear; 
close all;
import ETS3.*

%% Setting up the environment: you have to define YOUR ros domain id 
%getenv("ROS_DOMAIN_ID")
setenv("ROS_DOMAIN_ID","5")

%% Initializing a ros node
boat_node = ros2node("/Boat_node");
pause(3)

%% Creating publisher and subscribers
pause(3)
cmdvalPub = ros2publisher(boat_node, "/cmd_vel" ,"geometry_msgs/Twist");
%recieveSub = ros2subscriber(boat_node, %% INFORMATION FROM THE BOAT ABOUT POSITIONS );

%Defining message for publisher cmd_vel
cmdvelMsg = ros2message(cmdvalPub);

%% Defining variables
start = [-5 7 3/2*pi];
goal = [5 5 3/2*pi];
N=5;        


%% Lattice Planer Script
lp = Lattice();
lp.plan('iterations', 12)

lp.plan('cost',[1 10 10])  


%% Insert navigation strategy here


% Lattice planner navigation
% for R = 1:N
% 
% lp.query(start,goal)
% lp.plot
% hold on
% 
% start = goal;
% goal = goal+[-10 -2 0];
% 
% lp.query(start,goal)
% 
% pause(2)
% 
% lp.plot
% 
% start = goal;
% goal = goal+[10 -2 0];
% 
% pause(2)
% 
% end




%How to break down lattisplanner to cmdvelmsg?

cmdvelMsg.linear.x = 0.2;
cmdvelMsg.linear.y = 0.0;
cmdvelMsg.linear.z = 0.0;
cmdvelMsg.angular.x = 0.0;
cmdvelMsg.angular.y = 0.0;
cmdvelMsg.angular.z = 0.1;

for cnt = 1:30
send(cmdvalPub, cmdvelMsg)
 pause(1)
 disp("test")   
end