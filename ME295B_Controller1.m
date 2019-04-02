%ME295B script
%Using ManipulatorDynamics from RobotUtils the dynamics were derived for a
%3DOF manipulator
clear;
clc;
close all;
l0=0.5;
l1=0.5;
l2 =0.3;

r0 = 0.25;
r1 = 0.25;
r2 = 0.15;

Ix1 = 0.0050;
Ix2 = 0.0022;
Ix3 = 0.0104; 

Iy1 = 0.0050;
Iy2 = 0.0022;
Iy3 = 0.0104;

Iz1 = 0.0050;
Iz2 = 0.0022;
Iz3 = 0.0104;

m1 = 0.5;
m2 = 0.5;
m3 = 0.3;

g=9.81;
c1=0.001;
c2=0.001;
c3=0.001;

R1=0.6;
R2=0.4;
R3=0.2;

Kt1=3.5;
Kt2=2.5;
Kt3=1.9;

Kv1=3;
Kv2=2.6;
Kv3=2;

K1= R1/Kt1*(l1*m3*r2);
K2= R2/Kt2*(l1*m3*r2);
K3 = R3/Kt3*(l1*m3*r2);
K4 = R2/Kt2*g*l1*m3;
K5 = R1/Kt1*( (m3*(l1^2)) +(m2*(r1^2)) +Iz2);
K6 = R1/Kt1*( (m3*r2^2) +Iz3);
K7 = R1/Kt1*Iy2;
K8 = R1/Kt1*Iy3;
K9 = R1/Kt1*Iz1;
K10 = R2/Kt2*g*m2*(l0-r0);
K11 = R2/Kt2*g*m3*(l0-r0);
K12 = R2/Kt2*( (m3*(r2^2))+Ix3);
K13 = R2/Kt2*( (m3*(l1^2))+(m2*(r1^2))+(m3*(r2^2))+Ix2 +Ix3);
K14 = (Kv1+ (R1*c1/Kt1));
K15 = (Kv2+ (R2*c2/Kt2));
K16 = (Kv3+ (R3*c3/Kt3));

Phi = [K1; K2; K3; K4; K5; K6; K7; K8; K9; K10; K11; K12; K13; K14; K15; K16];

%% Uncertain values and bounds

%percent uncertainty
unc = 0.10;
zmin = 1 - unc;

z = zmin + rand(1,length(Phi)).*(unc*2);

%each Phi_hat_o is multipled by a random uncertainty
Phi_hat_o = Phi.*z';

%the bounds of max/min adaptive parameters is +/- boundsratio based on our
%initial guess
BoundsRatio = 0.20;
Phi_hat_max = (1+BoundsRatio)*Phi_hat_o;
Phi_hat_min = (1-BoundsRatio)*Phi_hat_o;

%% Controller gains

%Adaptive gains
Ka = 10.*diag(ones(1,length(Phi)));

%controller parameter
p=10/2;
 Eta = (p*2).*diag(ones(1,3));
%Eta = 10.*diag([1,1]);
%error parameter
 Lambda = (p/2).*diag(ones(1,3));
% Lambda = 8.*diag([1,1]);
%Lambda = 3.*diag([1,1]);

f = pi/2;
freq = [f f*2 f*3];
amp = [pi/4 pi/8 pi/5];

%% Sim and plots
sim('ME295B_Controller1Sim');

Phi_A = LineMatrix(Phi, t_out);
Phi_hat_Max = LineMatrix(Phi_hat_max, t_out);
Phi_hat_Min = LineMatrix(Phi_hat_min, t_out);

figure(1)
plot(t_out, E_out);
ylabel('Angle (rad)');
xlabel('Time (sec)');
title('Adaptive Controller Error vs Time');
legend('\theta_1', '\theta_2', '\theta3');

figure(2)
plot(t_out, Theta_out(:,1));
hold on;
plot(t_out, Theta_d_out(:,1));
ylabel('Angle (rad)');
xlabel('Time (sec)');
title('Adaptive Controller Angle vs time');
legend('\theta_1', '\theta_d_1');

figure(3)
plot(t_out, Theta_out(:,2));
hold on;
plot(t_out, Theta_d_out(:,2));
ylabel('Angle (rad)');
xlabel('Time (sec)');
title('Adaptive Controller Angle vs Time');
legend('\theta_2', '\theta_d_2');

figure(4)
plot(t_out, Theta_out(:,3));
hold on;
plot(t_out, Theta_d_out(:,3));
ylabel('Angle (rad)');
xlabel('Time (sec)');
title('Adaptive Controller Angle vs Time');
legend('\theta_3', '\theta_d_3');

figure(5)
plot(t_out, Phi_hat_out(:,1)', 'r');
hold on
title('Adaptation of K1');
xlabel('Time (sec)');
ylabel('Ohms A m^2');
plot(t_out, Phi_A(1,:), 'k--');
plot(t_out, Phi_hat_Min(1,:), 'k-.');
plot(t_out, Phi_hat_Max(1,:), 'k-.');
legend('Adaptation', 'Actual', 'Bounds');

figure(6)
plot(t_out, Phi_hat_out(:,2)', 'r');
hold on
plot(t_out, Phi_A(2,:), 'k--');
plot(t_out, Phi_hat_Min(2,:), 'k-.');
plot(t_out, Phi_hat_Max(2,:), 'k-.');
title('Adaptation of K2');
xlabel('Time (sec)');
ylabel('Ohms A m^2');
legend('Adaptation','Actual','Bounds','Location', 'SouthEast');

figure(7)
plot(t_out, Phi_hat_out(:,3)', 'r');
hold on
plot(t_out, Phi_A(3,:), 'k--');
plot(t_out, Phi_hat_Min(3,:), 'k-.');
plot(t_out, Phi_hat_Max(3,:), 'k-.');
title('Adaptation of K3');
xlabel('Time (sec)');
ylabel('Ohms A m^2');
legend('Adaptation','Actual','Bounds','Location', 'SouthEast');

figure(8)
plot(t_out, Phi_hat_out(:,4)', 'r');
hold on
plot(t_out, Phi_A(4,:), 'k--');
plot(t_out, Phi_hat_Min(4,:), 'k-.');
plot(t_out, Phi_hat_Max(4,:), 'k-.');
title('Adaptation of K4');
xlabel('Time (sec)');
ylabel('Ohms A m^2');
legend('Adaptation','Actual','Bounds', 'Location', 'East');

figure(9)
plot(t_out, Phi_hat_out(:,5)', 'r');
hold on
plot(t_out, Phi_A(5,:), 'k--');
plot(t_out, Phi_hat_Min(5,:), 'k-.');
plot(t_out, Phi_hat_Max(5,:), 'k-.');
title('Adaptation of K5');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{Ohms\:A\:sec}{rad}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'NorthEast');

figure(10)
plot(t_out, Phi_hat_out(:,6)', 'r');
hold on
plot(t_out, Phi_A(6,:), 'k--');
plot(t_out, Phi_hat_Min(6,:), 'k-.');
plot(t_out, Phi_hat_Max(6,:), 'k-.');
title('Adaptation of K6');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{Ohms\:A\:sec}{rad}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'NorthEast');

figure(11)
plot(t_out, Phi_hat_out(:,7)', 'r');
hold on
plot(t_out, Phi_A(7,:), 'k--');
plot(t_out, Phi_hat_Min(7,:), 'k-.');
plot(t_out, Phi_hat_Max(7,:), 'k-.');
title('Adaptation of K7');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{V\:sec}{rad}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(12)
plot(t_out, Phi_hat_out(:,8)', 'r');
hold on
plot(t_out, Phi_A(8,:), 'k--');
plot(t_out, Phi_hat_Min(8,:), 'k-.');
plot(t_out, Phi_hat_Max(8,:), 'k-.');
title('Adaptation of K8');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{V\:sec}{rad}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(13)
plot(t_out, Phi_hat_out(:,9)', 'r');
hold on
plot(t_out, Phi_A(9,:), 'k--');
plot(t_out, Phi_hat_Min(9,:), 'k-.');
plot(t_out, Phi_hat_Max(9,:), 'k-.');
title('Adaptation of K9');
xlabel('Time (sec)');
ylabel('kg^2 m^2 / s^2');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(14)
plot(t_out, Phi_hat_out(:,10)', 'r');
hold on
plot(t_out, Phi_A(10,:), 'k--');
plot(t_out, Phi_hat_Min(10,:), 'k-.');
plot(t_out, Phi_hat_Max(10,:), 'k-.');
title('Adaptation of K10');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(15)
plot(t_out, Phi_hat_out(:,11)', 'r');
hold on
plot(t_out, Phi_A(11,:), 'k--');
plot(t_out, Phi_hat_Min(11,:), 'k-.');
plot(t_out, Phi_hat_Max(11,:), 'k-.');
title('Adaptation of K11');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(16)
plot(t_out, Phi_hat_out(:,12)', 'r');
hold on
plot(t_out, Phi_A(12,:), 'k--');
plot(t_out, Phi_hat_Min(12,:), 'k-.');
plot(t_out, Phi_hat_Max(12,:), 'k-.');
title('Adaptation of K12');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(17)
plot(t_out, Phi_hat_out(:,13)', 'r');
hold on
plot(t_out, Phi_A(13,:), 'k--');
plot(t_out, Phi_hat_Min(13,:), 'k-.');
plot(t_out, Phi_hat_Max(13,:), 'k-.');
title('Adaptation of K13');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(18)
plot(t_out, Phi_hat_out(:,14)', 'r');
hold on
plot(t_out, Phi_A(14,:), 'k--');
plot(t_out, Phi_hat_Min(14,:), 'k-.');
plot(t_out, Phi_hat_Max(14,:), 'k-.');
title('Adaptation of K14');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(19)
plot(t_out, Phi_hat_out(:,15)', 'r');
hold on
plot(t_out, Phi_A(15,:), 'k--');
plot(t_out, Phi_hat_Min(15,:), 'k-.');
plot(t_out, Phi_hat_Max(15,:), 'k-.');
title('Adaptation of K15');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');

figure(20)
plot(t_out, Phi_hat_out(:,16)', 'r');
hold on
plot(t_out, Phi_A(16,:), 'k--');
plot(t_out, Phi_hat_Min(16,:), 'k-.');
plot(t_out, Phi_hat_Max(16,:), 'k-.');
title('Adaptation of K16');
xlabel('Time (sec)');
ylabel('$\mathrm{\frac{kg^2\:m^2}{s^2}}$',...
    'Interpreter', 'latex', 'FontSize', 17);
legend('Adaptation','Actual','Bounds','Location', 'East');