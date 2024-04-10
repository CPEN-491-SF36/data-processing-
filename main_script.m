%  Daniel Kawano, Rose-Hulman Institute of Technology
%  Last modified:  Dec 29, 2017
%
%  Thanks to Prithvi Akella (UC Berkeley) for helping with this code.

clear all
close all
clc


%  Load the raw iPhone sensor data obtained using MATLAB Mobile:

load('acceleration.mat');  % Load your acceleration data
load('angular_velocity.mat');  % Load your angular velocity data


%  Crop the data to begin just before the toss. This helps alleviate
%  excessive integration drift:

%crop = 0;

% %time stamping - normalize it 
% tdata_dt = Acceleration.Timestamp;           %  s
% tdata = datenum(tdata_dt);
% tdata_normal = tdata - tdata(1);
% 
% %angular velocity 
% %omega1 = omega1(crop:end);          %  rad/s
% omega1 = AngularVelocity.X;
% omega2 = AngularVelocity.Y;          %  rad/s
% omega3 = AngularVelocity.Z;          %  rad/s
% 
% %acceleration 
% %f1 = f1(crop:end);                  %  m/s^2
% f1 = Acceleration.X;
% f2 = Acceleration.Y;                  %  m/s^2
% f3 = Acceleration.Z;                  %  m/s^2    
% 
% clear all
% close all
% clc



% Load acceleration and angular velocity data
load('acceleration.mat');  % Load your acceleration data
load('angular_velocity.mat');  % Load your angular velocity data

% Assign data to variables
%time stamping - normalize it 
timestamp_acceleration = acceleration(:, 1);  % Assuming timestamp is in the first column
tdata = datenum(timestamp_acceleration);
tdata_normal = tdata - tdata(1);

f1 = acceleration(:, 2);  % Extract acceleration data from the second column
f2 = acceleration(:, 3);  % Extract acceleration data from the third column
f3 = acceleration(:, 4);  % Extract acceleration data from the fourth column

omega1 = angular_velocity(:, 2);  % Extract angular velocity data from the second column
omega2 = angular_velocity(:, 3);  % Extract angular velocity data from the third column
omega3 = angular_velocity(:, 4);  % Extract angular velocity data from the fourth column

%  Specify the initial conditions and simulation parameters:
% (Rest of the script remains unchanged)


%  Specify the initial conditions and simulation parameters:

psi0 = 0;           %  rad
theta0 = 0;       	%  rad
phi0 = 0;         	%  rad

y0 = [psi0, theta0, phi0]';

x10 = 0;            %  m
x20 = 0;            %  m
x30 = 0;            %  m

x1dot0 = 0;         %  m/s
x2dot0 = 0;        	%  m/s
x3dot0 = 0;       	%  m/s

tol = 1e-6;

%  Numerically integrate the first-order (state) equations governing the 
%  iPhone's orientation:

options = odeset('abstol', tol, 'reltol', tol);

%[t, y] = ode45(@iphone_orientation_ODEs, tdata, y0, options, omega1, omega2, omega3);
%ydot = iphone_orientation_ODEs(t, y, omega1, omega2, omega3, tdata)
%[t, y] = ode45(@(t,y) iphone_orientation_ODEs(t, y, omega1, omega2, omega3, tdata), tdata, y0);
[t, y] = ode45(@iphone_orientation_ODEs, tdata_normal, y0, options, omega1, omega2, omega3, tdata_normal);

%  Extract the results:

psi = y(:,1);           %  rad
theta = y(:,2);         %  rad
phi = y(:,3);           %  rad           
           
%  Transform the specific force data into space-fixed components using the
%  computed 3-2-1 Euler angles:

for k = 1:length(psi)
    R1 = [ cos(psi(k)), sin(psi(k)), 0;                          
          -sin(psi(k)), cos(psi(k)), 0;              
           0, 0, 1];
    R2 = [cos(theta(k)), 0, -sin(theta(k));       
          0, 1, 0;
          sin(theta(k)), 0, cos(theta(k))];    
    R3 = [1, 0, 0;                               
          0, cos(phi(k)), sin(phi(k));       
          0, -sin(phi(k)), cos(phi(k))];
    F123 = ((R3*R2*R1)')*[f1(k), f2(k), f3(k)]';        %  m/s^2    
    F1(k,1) = F123(1);                                  %  m/s^2
    F2(k,1) = F123(2);                                  %  m/s^2
    F3(k,1) = F123(3);                                  %  m/s^2
end

%  Subtract away the mean from the space-fixed components of the specific 
%  force to correct for gravitational acceleration and center the signals
%  about 0. This zero-centering alleviates the effect of sensor bias and 
%  improves the forthcoming displacement results:

x1ddot = F1 - mean(F1);        %  m/s^2
x2ddot = F2 - mean(F2);        %  m/s^2
x3ddot = F3 - mean(F3);        %  m/s^2

%  Numerically integrate the space-fixed acceleration components to obtain 
%  the velocity components of the iPhone's IMU:

x1dot = cumtrapz(tdata_normal, x1ddot) + x1dot0;       %  m/s
x2dot = cumtrapz(tdata_normal, x2ddot) + x2dot0;      	%  m/s
x3dot = cumtrapz(tdata_normal, x3ddot) + x3dot0;    	%  m/s

%  Numerically integrate the space-fixed velocity components to obtain the
%  components of displacement:

x1 = cumtrapz(tdata_normal, x1dot) + x10;    	%  m
x2 = cumtrapz(tdata_normal, x2dot) + x20;    	%  m
x3 = cumtrapz(tdata_normal, x3dot) + x30;    	%  m

%  Truncate the results to just after the iPhone is caught on its way down:

[~, index] = max(diff(x3ddot));
index = index + 5;

t = t(1:index);                 %  s
dt = t(2) - t(1);               %  s

psi = psi(1:index);             %  rad
theta = theta(1:index);         %  rad
phi = phi(1:index);             %  rad

x1 = x1(1:index);               %  m
x2 = x2(1:index);               %  m
x3 = x3(1:index);               %  m

%  Plot the Euler angles over time:

figure
set(gcf, 'color', 'w', 'name', 'Euler angles over time')
subplot(311)
plot(t, psi*(180/pi), '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\it\psi\rm (deg)')
subplot(312)
plot(t, theta*(180/pi), '-r', 'linewidth', 2)
hold on
plot([t(1), t(end)], [1, 1]*90, '--k', ...
     [t(1), t(end)], [1, 1]*(-90), '--k')
ylim(100*[-1, 1])
xlabel('Time (s)')
ylabel('\it\theta\rm (deg)')
subplot(313)
plot(t, phi*(180/pi), '-k', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\it\phi\rm (deg)')

%  Plot the displacement of the iPhone's IMU over time:

figure
set(gcf, 'color', 'w', 'name', 'IMU displacement over time')
subplot(311)
plot(t, x1, '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_1 (cm)')
subplot(312)
plot(t, x2, '-r', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_2 (cm)')
subplot(313)
plot(t, x3, '-k', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_3 (cm)')

%  Animate the iPhone's motion:

figure
set(gcf, 'color', 'w', 'name', 'IMU displacement over time in xy direction ')
subplot(311)
plot(x1, x2, '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_1 (cm)')


animate_iphone(x1, x2, x3, psi, theta, phi, dt, 3, 2, 1, 'default');