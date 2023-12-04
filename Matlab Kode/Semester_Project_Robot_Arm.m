%% Script for Robot Arm

% Clearing variables
clc; clear; close all;

%% 

import ETS3.*

% Setting up environment
setenv("ROS_DOMAIN_ID","5");
% Initializing ros node
armControllerNode = ros2node("/open_manipulator_controller");

%trajPub = ros2publisher(armControllerNode, "/arm_controller/joint_trajectory", "trajectory_msgs/JointTrajectory");
trajPub = ros2publisher(armControllerNode, "/joint_trajectory/joint_trajectory","trajectory_msgs/JointTrajectory")
pause(3)

% Defining message for publisher
trajMsg = ros2message(trajPub);
trajMsg.joint_names = {'arm_base_joint', 'link_1_joint', 'link_2_joint', 'link_3_joint','gripper_base_joint'};


%% Defining the robotic arm DH parameters

% offset = [pi,pi/2+beta,pi/2-beta,0];

L1 = 0.200;
L2 = 0.385;
L3 = 0.270;
L4 = 0.115;
L5 = 0.200;

%L6 = sqrt(L2*L2 + L3*L3);
%beta = atan(L3/L2);

j1 = Revolute('d', L1, 'a', 0, 'alpha', pi/2, 'offset', 0);
j2 = Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', 0);
j3 = Revolute('d', 0, 'a', L3, 'alpha', 0, 'offset', 0);
j4 = Revolute('d', 0, 'a', L4, 'alpha', 0, 'offset', 0);
j5 = Revolute('d', 0, 'a', L5, 'alpha', 0, 'offset', 0);

robot = SerialLink([j1 j2 j3 j4 j5],'name', 'my robot');

%% Use the calculations from the arm kinematics lecture.

robot.qlim = [-3.14, +3.14; -1.57, +1.57; -1.40, +1.57; -1.57, 1.57; -3.14, +3.14];
robot.teach

% Visualizing the arm on zero position to check that the definition is correct
robot.plot([0, 0, 0, 0, 0]);


% Go first to "zero" point 
point1 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point1.positions =  [0.0, 0.0, 0.0, 0.0, 0.0];
point1.velocities = [0, 0, 0, 0, 0];
point1.accelerations = [0, 0, 0, 0, 0];
point1.effort = [0, 0, 0, 0, 0];
point1.time_from_start.sec = int32(2);

% Check in the plot that it looks okay

robot.plot(point1.positions);

%%

global Pvec

testVector = Pvec/100

%T_robot_goal_1 = SE3(0.35, 0.0 , -0.01) * SE3.rpy(0,0,0, 'deg');
%T_robot_goal_1 = SE3(testVector(1),testVector(2),testVector(3)) * SE3.rpy(0,0,0, 'deg');
q1 = robot.ikcon(T_robot_goal_1, point1.positions) % this takes into account joint limits and the inital position

% Check in the plot that it looks okay

robot.plot(q1)


%%
point2 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point2.positions =  q1;
point2.velocities = [0, 0, 0, 0, 0];
point2.accelerations = [0, 0, 0, 0, 0];
point2.effort = [0, 0, 0, 0, 0];
point2.time_from_start.sec = int32(5);
trajMsg.points = [point1, point2];
send(trajPub, trajMsg)