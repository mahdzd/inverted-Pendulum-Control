%% Furuta Pendulum - Qube Servo 3
% Parameter and state-space definition script

%% Physical parameters (from Qube-Servo 3 manual)
mr  = 0.095;      % rotary arm mass [kg]
Lr  = 0.085;      % rotary arm length pivot->end [m]
mp  = 0.024;      % pendulum link mass [kg]
Lp  = 0.129;      % pendulum link length [m]

Jm  = 1.4e-6;     % motor rotor inertia [kg*m^2]
Jh  = 0.6e-6;     % hub inertia [kg*m^2]

Rm  = 7.5;        % motor terminal resistance [Ohm]
Kt  = 0.0422;     % torque constant [N*m/A]
Km  = 0.0422;     % back-EMF constant [V/(rad/s)]

g   = 9.81;       % gravity [m/s^2]

%% Extra modeling assumptions
br  = 0.001;      % rotary arm viscous friction [N*m*s/rad] (approx)
bp  = 0.00005;    % pendulum viscous friction [N*m*s/rad] (approx)

r   = Lr;         % arm COM distance (rod from pivot)
hg  = Lp/2;       % pendulum COM distance from pivot (uniform rod)

% Inertias about pivot
Jr  = Jm + Jh + (1/3)*mr*Lr^2;   % total rotary arm inertia about pivot
Jp  = (1/3)*mp*Lp^2;             % pendulum inertia about pivot

%% Compact constants (as in derivation)
A1  = Jr + mr*r^2;
A2  = Jp + mp*hg^2;
B   = mr*r*hg;

Delta = A1*A2 - B^2;             % determinant term A1*A2 - B^2

%% State-space matrices (input u = Vm)
% State: x = [theta; alpha; theta_dot; alpha_dot]

A = [ 0,                               0,                             1,                           0;
      0,                               0,                             0,                           1;
      0,              -B*mp*g*hg/Delta, -A2*(br + Km^2/Rm)/Delta,   B*bp/Delta;
      0,               A1*mp*g*hg/Delta,  B*(br + Km^2/Rm)/Delta,  -A1*bp/Delta ]

Bmat = [ 0;
         0;
         A2*(Km/Rm)/Delta;
        -B*(Km/Rm)/Delta ]

C = [1 0 0 0;   % measure theta
     0 1 0 0];  % measure alpha

D = [0;
     0];

% (Optional) create state-space object:
 sys = ss(A, Bmat, C, D);
 G = tf(sys)

 % Given A and B matrices:

 % --- Ackermann full state feedback ---
p = [-300 -040 -001.2+012.5j -001.2-0.125j];
 K = acker(A, Bmat, p)

 % Verify closed-loop
