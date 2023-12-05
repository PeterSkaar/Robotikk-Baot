%% Appendiks G
%% Matlab script for transformasjonsmatriser robot arm

syms L1 L2 L3 L4 L5 q1 q2 q3 q4

% DH parameter
j1 = Revolute('d', L1,        'a', 0,   'alpha', pi/2 );
j2 = Revolute('d', 0,         'a', L2,  'alpha', 0, 'offset', pi/2      );
j3 = Revolute('d', 0,         'a', L3,  'alpha', 0                      );
j4 = Revolute('d', 0,         'a', L4,  'alpha', pi/2                   );
j5 = Revolute('d', L5,        'a', 0,  'alpha' , 0                      );

Robot = SerialLink([j1 j2 j3 j4 j5], 'name', 'Le Baot');


% Transformasjonsmatrise frå origo båt {B} til arm base {O}
Transformasjon_B0 = transl(20,0,20)

% Transformasjonsmatrise frå arm base {0} til link {1}
Transformasjon_01 = SerialLink(j1).fkine(q1)

% Transformasjonsmatrise frå {1} til {2}
Transformasjon_12 = SerialLink(j2).fkine(q2)

% Transformasjonsmatrise frå {2} til {3}
Transformasjon_23 = SerialLink(j3).fkine(q3)

% Transformasjonsmatrise frå {3} til {4}
Transformasjon_34 = SerialLink(j4).fkine(q4)

% Transformasjonsmatrise frå {4} til {5}
Transformasjon_45 = SerialLink(j5).fkine(0)

% Transformasjonsmatrise frå arm base {0} til endeffector {5}
Transformasjon_05 = Robot.fkine([th1, th2, th3, th4, 0])



% Kamera 1
C1x = 32.5;
C1y = 18;
C1z = 22;

% Kamera 2
C2x = 32.5;
C2y = -18;
C2z = 22;

% Fast rotasjon hentet frå URDF i Gazebo
RC1 = [0.9888, -0.1490, 0, 0; 0.1490, 0.9888 0, 0; 0 0 1 0; 0 0 0 1]
RC2 = [0.9888, 0.1490, 0, 0; -0.1490, 0.9888 0, 0; 0 0 1 0; 0 0 0 1]

% Transformasjonsmatrise frå origo båt {B} til kamera {C1 og C2}
Transformasjon_BC1 = transl(C1x, C1y, C1z) * RC1
Transformasjon_BC2 = transl(C2x, C2y, C2z) * RC2

Transformasjon_C1B = inv(Transformasjon_BC1)
Transformasjon_C2B = inv(Transformasjon_BC2)

% Transformasjonsmatrise frå kamera {C1 og C2} til base arm {0}
Transformasjon_C10 = Transformasjon_C1B * Transformasjon_B0
Transformasjon_C20 = Transformasjon_C2B * Transformasjon_B0
