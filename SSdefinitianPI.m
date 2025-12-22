
clear; clc;

%% ===== Parameters (edit as needed) =====
mr  = 0.095;      % rotary arm mass [kg]
Lr  = 0.085;      % rotary arm length [m]
mp  = 0.024;      % pendulum mass [kg]
Lp  = 0.129;      % pendulum length [m]

Jm  = 1.4e-6;     % motor rotor inertia [kg*m^2]
Jh  = 0.6e-6;     % hub inertia [kg*m^2]

Rm  = 7.5;        % motor terminal resistance [Ohm]
Kt  = 0.0422;     % torque constant [N*m/A]
Km  = 0.0422;     % back-EMF constant [V/(rad/s)]

g   = 9.81;       % gravity [m/s^2]
br  = 0.001;      % arm viscous damping [N*m*s/rad]
bp  = 0.00005;    % pendulum viscous damping [N*m*s/rad]

%% ===== Derived geometry/inertias =====
r  = Lr;
hg = Lp/2;

jr = (1/3)*mr*Lr^2 ;    % effective rotary inertia about pivot`
jp = (1/3)*mp*Lp^2;              % pendulum inertia about its COM (as stated)

Jt     = jr ;
Jalpha = jp ;

B = mp*r*hg;   % coupling term

%% ===== Linearized equations in matrix form =====
% M * [th_dd; al_dd] = [ tau - br*th_d ;
%                       -bp*al_d - mp*g*hg*al ]
M = [ Jt,     B;
    B,  Jalpha ];

Minv = inv(M);

%% ===== State vector =====
% x = [ theta; alpha_tilde; theta_dot; alpha_dot ]
% u = tau_m  (torque input)
A = zeros(4,4);
B_tau = zeros(4,1);

% xdot1 = theta_dot
A(1,3) = 1;

% xdot2 = alpha_dot
A(2,4) = 1;


A(3,2) = Minv(1,2)*(-mp*g*hg);   % from -mp*g*hg*alpha
A(3,3) = Minv(1,1)*(-br);        % from -br*theta_dot
A(3,4) = Minv(1,2)*(-bp);        % from -bp*alpha_dot
B_tau(3) = Minv(1,1);            % from u

% alpha_ddot coefficients
A(4,2) = Minv(2,2)*(-mp*g*hg);
A(4,3) = Minv(2,1)*(-br);
A(4,4) = Minv(2,2)*(-bp);
B_tau(4) = Minv(2,1);

%% ===== Output matrices  =====
C = [ 1 0 0 0 ;
      0 1 0 0 ];

D = zeros(2,1);


sys_tau = ss(A, B_tau, C, D);

disp('A ='); disp(A);
disp('B (tau input) ='); disp(B_tau);
disp('C ='); disp(C);
disp('D ='); disp(D);


B_V = B_tau * (Kt/Rm);                 % input gain to Vm
A_V = A + B_tau * (-(Kt*Km/Rm)) * [0 0 1 0];   % add extra term multiplying theta_dot

sys_V = ss(A_V, B_V, C, D);

disp('--- Using Vm as input ---');
disp('A_V ='); disp(A_V);
disp('B_V ='); disp(B_V);
