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

syms th1 th2 th3 th4 L1 L2 L3 L4 L5

% Utrekningar
% Base robotarm
T0 = transl(20,0,20)

R1 = trotz(th1);
V1 = transl(0,0,L1);
T1 = T0*V1*R1

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
Transfomrasjon = simplify(Transformasjon)

% Kamera symbolsk

syms C1x C1y C1z C2x C2y C2z q1 q2

% Fast rotasjon hentet frå rpy2rad
% R = rpy2r(0,0,pi/21)

RC1 = [0.9888, -0.1490, 0, 0; 0.1490, 0.9888 0, 0; 0 0 1 0; 0 0 0 1]
RC2 = [0.9888, 0.1490, 0, 0; -0.1490, 0.9888 0, 0; 0 0 1 0; 0 0 0 1]

C1 = transl(C1x, C1y, C1z) * RC1
C2 = transl(C2x, C2y, C2z) * RC2

Transformasjon_C1W = inv(C1)
Transformasjon_C2W = inv(C2)

Transformasjon_C1Arm = Transformasjon_C1W *T0
Transformasjon_C2Arm = Transformasjon_C2W *T0

% Kamera til base arm
vpa(Transformasjon_C1Arm,4)
vpa(Transformasjon_C2Arm,4)

% Kamera til end effector

Transformasjon_C1EE = Transformasjon_C1W *T5
Transformasjon_C2EE = Transformasjon_C2W *T5

C1_EE = simplify(Transformasjon_C1EE)
C2_EE = simplify(Transformasjon_C2EE)

C1_EE = vpa(Transformasjon_C1EE,4)
C2_EE = vpa(Transformasjon_C2EE,4)

%% Matriser: Transformasjons med verdier

clc
clear all

% Still theta verdier om ønskelig
th1 = deg2rad(10);
th2 = deg2rad(10);
th3 = deg2rad(10);
th4 = deg2rad(10);

M = eye(4)

% Båt variablar [cm]

breidde = 40;
lengde = 100;
hoyde = 20;

% Robot arm lengder [cm]
L1 = 5;
L2 = 34.4;
L3 = 21.26;
L4 = 15.7;
L5 = 0;
off_end = 5; % offset endeffector

% Utrekningar
% Base robotarm
T0 = transl(20,0,20)


R1 =   trotz(th1);
V1 =   transl(0,0,L1);
T1 =   T0*V1*R1

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
% Antar at robotarm starter i x,y,z = 0

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


%% Kamera posisjonar

% Verdier for Kamera 1
C1x = 32.5;
C1y = 18;
C1z = 22;

% Verdier for Kamera 2
C2x = 32.5;
C2y = -18;
C2z = 22;

RC1 = trotz(15,'deg')
RC2 = trotz(15,'deg')

C1 = transl(C1x, C1y, C1z) * RC1
C2 = transl(C2x, C2y, C2z) * RC2

Transformasjon_C1W = inv(C1)
Transformasjon_C2W = inv(C2)

Transformasjon_C1Arm = Transformasjon_C1W *T0;
Transformasjon_C2Arm = Transformasjon_C2W *T0;

% Plott for å vise kamera plassering

coor_Scope = [ -20,  20;   %y
                0 , 20;   %z
               -50,  50;]; %x
A=[coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,2);
   coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,2);
   coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,2);
   coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,2)];
d=[1 2 3 4 8 5 6 7 3 2 6 5 1 4 8 7];
X = A(d,3);
Y = A(d,1);
Z = A(d,2);

hold on;
plot3(A(d,3),A(d,1),A(d,2));
hold on;
patch([X(1:6) flip(X(1:6))], [Y(1:6) flip(Y(1:6))], [Z(1:6) flip(Z(1:6))], 'r', 'FaceAlpha',0.25)
kp = 2;
patch([X((1:6)+kp) flip(X((1:6)+kp))], [Y((1:6)+kp) flip(Y((1:6)+kp))], [Z((1:6)+kp) flip(Z((1:6)+kp))], 'g', 'FaceAlpha',0.25)
kp = 10;
patch([X((1:6)+kp) flip(X((1:6)+kp))], [Y((1:6)+kp) flip(Y((1:6)+kp))], [Z((1:6)+kp) flip(Z((1:6)+kp))], 'b', 'FaceAlpha',0.25)
hold on;
xlim([-80 80])
ylim([-80 80])
zlim([-80 80])
hold on;
trplot(M,  'World' , '1', 'color', 'b');
trplot(T0, 'Robotarm' , '2', 'color', 'g');
trplot(C1, 'kamera1', '1', 'color', 'r');
trplot(C2, 'kamera2', '2', 'color', 'r');


%% Plotting
plotvol([0 90 0 90 -5 20]);
trplot(M,  'frame' , '1', 'color', 'b');
trplot(T0, 'frame' , '1', 'color', 'b');
trplot(T1, 'frame' , '2', 'color', 'r');
trplot(T2, 'frame' , '3', 'color', 'g');
trplot(T4, 'frame' , '4', 'color', 'b');
trplot(T5, 'frame' , '5', 'color', 'r');
trplot(C1, 'kamera', '6', 'color', 'r');
trplot(C2, 'kamera', '7', 'color', 'r');


%% Animering

% Første transformasjons
title('Baot arm ved 10 grader på alle vinklar')
tranimate(T0, T1, '3D', 'axis', [0, 90, 0, 90, -5, 50])
legend('show');
hold on

% Plotter linje 1
translation0 = T0(1:3, 4);
translation1 = T1(1:3, 4);
plot3([translation0(1), translation1(1)], [translation0(2), translation1(2)], [translation0(3), translation1(3)], '-o', 'DisplayName', 'Link 1');
hold on

% Andre transformasjon
tranimate(T1, T2, '3D', 'axis', [0, 90, 0, 90, -5, 50])
hold on

% Plotter linje 2
translation2 = T2(1:3, 4);
plot3([translation1(1), translation2(1)], [translation1(2), translation2(2)], [translation1(3), translation2(3)], '-o', 'DisplayName', 'Link 2');
hold on

% Tredje transformasjon
tranimate(T2, T3, '3D', 'axis', [0, 90, 0, 90, -5, 50])
hold on

% Plotter linje 3
translation3 = T3(1:3, 4);
plot3([translation2(1), translation3(1)], [translation2(2), translation3(2)], [translation2(3), translation3(3)], '-o', 'DisplayName', 'Link 3');
hold on

% Fjerde transformasjon
tranimate(T3, T4, '3D', 'axis', [0, 90, 0, 90, -5, 50])

% Plotter linje 4
translation4 = T4(1:3, 4);
plot3([translation3(1), translation4(1)], [translation3(2), translation4(2)], [translation3(3), translation4(3)], '-o', 'DisplayName', 'Link 4');
hold on

% Femte transformasjon
tranimate(T4, T5, '3D', 'axis', [0, 90, 0, 90, -5, 50])
hold on

% Plotter linje 5
translation5 = T5(1:3, 4);
plot3([translation4(1), translation5(1)], [translation4(2), translation5(2)], [translation4(3), translation5(3)], '-o', 'DisplayName', 'Link 5');




