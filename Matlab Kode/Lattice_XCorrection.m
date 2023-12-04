
%% Test-Script
clc; 
clear; 
close all;

% Setting up environment
setenv("ROS_DOMAIN_ID","5");
% Initializing ros node
braitenberControllerNode = ros2node("/braitenberg_controller");

% Creating subscriber to odom and publisher to cmd velocity
pause(3)
odomSub = ros2subscriber(braitenberControllerNode,"/odom","nav_msgs/Odometry","Reliability","besteffort","Durability","volatile","Depth",5);
cmdvalPub = ros2publisher(braitenberControllerNode, "/cmd_vel", "geometry_msgs/Twist");
pause(3)

% Defining message for publisher
cmdvelMsg = ros2message(cmdvalPub);

% Defining variables
odomZ = 0;
errorZ = 5; %Tillatt feilmargin
errorX = 1; %Tillatt feilmargin
turn_speed = 0.4;       
forwardSpeed = 0.4;


start = [0 0];
goal =  [10 -2];

%% Error and disered z	
% desired_z = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
% err_z = self.normalize_angle(desired_yaw - self.yaw)
% 
% start = [0 0 3/2*pi];
% goal =  [10 -2 3/2*pi];
% 
% desired_xAxis= abs(start(1))+abs(goal(1));
% desired_yAxis= abs(start(2))+goal(2);
% desired_pose= abs(start(3))+goal(3);
% 
% start = goal;
% goal = goal+[-10 -2 0];
% 
% desired_xAxis= abs(start(1))+abs(goal(1));
% desired_yAxis= abs(start(2))-goal(2);
% desired_pose= abs(start(3))+goal(3);

%% Get currentAngle

while true %While ikkje riktig vinkel

[scanData,status,statustext] = receive(odomSub,10);             %%Mottar data fra odom 
robotPosition = scanData.pose.pose.position                     %%Posisjonsdata.
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

pause(0.3)

quaternion = [x y z w]
euler = quat2eul(quaternion,'XYZ')
currentAngle = rad2deg(euler)
%self.yaw = euler[2]


angleZ = currentAngle(1) - 0 %desiredAngle;

if(angleZ > errorZ)
    cmdvelMsg.angular.z = turn_speed;
    disp("turning left");
end

   
if (angleZ < (0-errorZ))
    cmdvelMsg.linear.x = forwardSpeed;
    cmdvelMsg.angular.z = -turn_speed;
    disp("turnting right");
end


if (abs(angleZ) < errorZ) 
     cmdvelMsg.angular.z = 0;
     cmdvelMsg.linear.x = forwardSpeed;
     disp("Kjøyrer rett fram")
end
   

send(cmdvalPub, cmdvelMsg)


end




% 
% twist_msg = Twist()
% if math.fabs(err_yaw) > self.yaw_precision:
% twist_msg.angular.z = 0.3 if err_yaw > 0.0 else -0.3
% 
% 
% def normalize_angle(self, angle):
% f(math.fabs(angle) > math.pi):
% angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
% return angle
% 
%%

cmdvelMsg.linear.x = 0.0;
cmdvelMsg.linear.y = 0.0;
cmdvelMsg.linear.z = 0.0;
cmdvelMsg.angular.x = 0.0;
cmdvelMsg.angular.y = 0.0;
cmdvelMsg.angular.z = 0.0;
send(cmdvalPub,cmdvelMsg)
