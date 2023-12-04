%% Cleaning Variables and Command Window

clear all; close all; clc


%% Lattice Planer Script

lp = Lattice();
lp.plan('iterations', 14)

lp.plan('cost',[1 10 10])  

load road
about road


%init = [-5 7 3/2*pi];
goal = [5 5 3/2*pi];
init = [-5 7 3/2*pi];
%goal = [5 5]

x0 = init;
xg = goal;

% r = sim( 'sl_drivepoint');
% q = r.find('y');
% plot(q(:,1), q(:,2));

lp.plot

% lp.query(init,goal)
% lp.plot
% hold on

% init = goal;
% goal = goal+[-10 -2 0];
% 
% lp.query(init,goal)

% lp.plot

N=2;

for R = 1:N

lp.query(init,goal)
lp.plot
hold on



init = goal;
goal = goal+[-10 -2 0];
 
lp.query(init,goal)

pause(0.2)

lp.plot

init = goal;
goal = goal+[10 -2 0];

pause(0.2)
end




