%% Prosjekt Baot
% Transformasjonsmatriser symbolsk og med verdiar for BaotArm
% Prosjekt Haust 2023
% Peter Søreide Skaar, Vegard Aven Ullbenø, Roar Bøyum

% Scriptet er best køyrd i seksjonar basert på kva ein ønsker

%% Denavit-Hartenberg parameters / links & joints

L1 = 5;
L2 = 34.4;
L3 = 21.26;
L4 = 15.7;
L5 = 5;

j1 = Revolute('d', L1,        'a', 0,   'alpha', pi/2  );
j2 = Revolute('d', 0,         'a', L2,  'alpha', 0     );
j3 = Revolute('d', 0,         'a', L3,  'alpha', 0     );
j4 = Revolute('d', 0,         'a', L4,  'alpha', pi/2  );
j5 = Revolute('d', L5,        'a', 0,  'alpha' , 0     );

%% Robot
Robot = SerialLink([j1 j2 j3 j4 j5]);
Robot.teach

%% Matriser : Transformasjons symbolsk

syms th1 th2 th3 th4 L1 L2 L3 L4 off_end

M  = eye(4)

% Utrekningar
R1 = trotz(th1);
V1 = transl(0,0,L1);
T1 = M*V1*R1

% Snur heile feltet
T1 = T1*trotx(pi/2)

R2 = trotz(th2);
V2 = transl(L2,0,0);
T2 = T1*R2*V2

R3 = trotz(th3);
V3 = transl(L3,0,0);
T3 = T2 * R3 * V3
 
R4 = trotz(th4);
V4 = transl(L4,0,0);
T4 = T3 * R4 * V4
T4 = T4*trotx(pi/2)

V5 = transl(0,0,L5);
R5 = eye(4)
T5 = T4*R5*V5;

Transformasjon = T5
Transofmrasjon = simplify(Transformasjon)

%% Matriser: Transformasjons med verdier

clc
clear all

% Still theta verdier om ønskelig
th1 = deg2rad(10);
th2 = deg2rad(10);
th3 = deg2rad(10);
th4 = deg2rad(10);

% Robot arm lengder
L1 = 5;
L2 = 34.4;
L3 = 21.26;
L4 = 15.7;
L5 = 0;
off_end = 5; % offset endeffector

M  = eye(4)

% Utrekningar
R1 = trotz(th1);
V1 = transl(0,0,L1);
T1 = M*V1*R1

% Snur heile feltet
T1 = T1*trotx(pi/2)

R2 = trotz(th2);
V2 = transl(L2,0,0);
T2 = T1*R2*V2

R3 = trotz(th3);
V3 = transl(L3,0,0);
T3 = T2 * R3 * V3
 
R4 = trotz(th4);
V4 = transl(L4,0,0);
T4 = T3 * R4 * V4
T4 = T4*trotx(pi/2)

V5 = transl(0,0,L5);
R5 = eye(4)
T5 = T4*R5*V5;

Transformasjon = T5

%% Matriser: Forward Kinematics

% Still theta verdier om ønskelig
th1 = deg2rad(10);
th2 = deg2rad(10);
th3 = deg2rad(10);
th4 = deg2rad(10);

Transformasjon_01 = SerialLink(j1).fkine(th1)
Transformasjon_12 = SerialLink(j2).fkine(th2)
Transformasjon_23 = SerialLink(j3).fkine(th3)
Transformasjon_34 = SerialLink(j4).fkine(th4)
Transformasjon_45 = SerialLink(j5).fkine(0)

Transformasjon_05 = Robot.fkine([th1, th2, th3, th4, 0])

%% Plotting
plotvol([0 90 0 90 -5 20]);
trplot(M, 'frame', '1', 'color', 'b');
trplot(T1, 'frame', '2', 'color', 'r');
trplot(T2, 'frame', '3', 'color', 'g');
trplot(T4, 'frame', '4', 'color', 'b');
trplot(T5, 'frame', '5', 'color', 'r');

%% Animering

% Første transformasjons
title('Baot arm ved 10 grader på alle vinklar')
tranimate(M, T1, '3D', 'axis', [0, 90, 0, 90, -5, 40])
legend('show');
hold on

% Plotter linje 1
translation1 = T1(1:3, 4);
plot3([0, translation1(1)], [0, translation1(2)], [0, translation1(3)], '-o', 'DisplayName', 'Link 1');
hold on

% Andre transformasjon
tranimate(T1, T2, '3D', 'axis', [0, 90, 0, 90, -5, 40])
hold on

% Plotter linje 2
translation2 = T2(1:3, 4);
plot3([translation1(1), translation2(1)], [translation1(2), translation2(2)], [translation1(3), translation2(3)], '-o', 'DisplayName', 'Link 2');
hold on

% Tredje transformasjon
tranimate(T2, T3, '3D', 'axis', [0, 90, 0, 90, -5, 40])
hold on

% Plotter linje 3
translation3 = T3(1:3, 4);
plot3([translation2(1), translation3(1)], [translation2(2), translation3(2)], [translation2(3), translation3(3)], '-o', 'DisplayName', 'Link 3');
hold on

% Fjerde transformasjon
tranimate(T3, T4, '3D', 'axis', [0, 90, 0, 90, -5, 40])

% Plotter linje 4
translation4 = T4(1:3, 4);
plot3([translation3(1), translation4(1)], [translation3(2), translation4(2)], [translation3(3), translation4(3)], '-o', 'DisplayName', 'Link 4');
hold on

% Femte transformasjon
tranimate(T4, T5, '3D', 'axis', [0, 90, 0, 90, -5, 40])
hold on

% Plotter linje 5
translation5 = T5(1:3, 4);
plot3([translation4(1), translation5(1)], [translation4(2), translation5(2)], [translation4(3), translation5(3)], '-o', 'DisplayName', 'Link 5');




