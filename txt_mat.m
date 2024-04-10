% % Open the text file
% fid = fopen('mpu_11.txt', 'r');
% 
% % Initialize arrays to store data
% angular_velocity = [];
% acceleration = [];
% gp2 = [];
% lidar = [];
% 
% 
% % Read each line of the text file
% tline = fgetl(fid);
% while ischar(tline)
%     % Use textscan to parse the line
%     data = textscan(tline, 'MPU_ID: %*d Time: %f Gx=%f°/s Gy=%f°/s Gz=%f°/s Ax=%fg Ay=%fg Az=%fg');
% 
%     % Extract data from textscan result
%     time = data{1};
%     gx = data{2};
%     gy = data{3};
%     gz = data{4};
%     ax = data{5};
%     ay = data{6};
%     az = data{7};
% 
%     % Append timestamp to angular velocity and acceleration data
%     angular_velocity(end+1,:) = [time, gx, gy, gz];
%     acceleration(end+1,:) = [time, ax, ay, az];
% 
%     % Read the next line
%     tline = fgetl(fid);
% end
% % Close the text file
% fclose(fid);
% 
% % Save angular velocity and acceleration data into separate .mat files
% save('angular_velocity.mat', 'angular_velocity');
% save('acceleration.mat', 'acceleration');


clear all
close all
clc


% Define file names
file_names = {'mpu_11.txt', 'gp2_11.txt', 'lidar_11.txt'};

% Initialize arrays to store data
angular_velocity = [];
acceleration = [];
gp2 = [];
lidar = [];

% Loop through each file
for i = 1:numel(file_names)
    % Open the text file
    fid = fopen(file_names{i}, 'r');
    
    if fid == -1
        error(['Unable to open file ' file_names{i}]);
    end

    % Read the first line of the text file
    tline = fgetl(fid);
    while ischar(tline)
        % Extract data based on the sensor ID
        if contains(tline, 'MPU_ID')
            % Extract angular velocity and acceleration data
            data = textscan(tline, 'MPU_ID: %*d Time: %f Gx=%f°/s Gy=%f°/s Gz=%f°/s Ax=%fg Ay=%fg Az=%fg');
            time = data{1};
            gx = data{2};
            gy = data{3};
            gz = data{4};
            ax = data{5};
            ay = data{6};
            az = data{7};

            % Append timestamp to angular velocity and acceleration data
            angular_velocity(end+1,:) = [time, gx, gy, gz];
            acceleration(end+1,:) = [time, ax, ay, az];
        elseif contains(tline, 'GP2_ID: 1')
            % Extract GP2 sensor data with ID 1
            data = textscan(tline, 'GP2_ID: %*d Distance: %f Time: %f');
            distance = data{1};
            time = data{2};

            % Append timestamp to GP2 data with ID 1
            gp2(end+1,:) = [time, distance];
        elseif contains(tline, 'LIDAR_ID: 1')
            % Extract LIDAR sensor data with ID 1
            data = textscan(tline, 'LIDAR_ID: %*d Distance: %f Time: %f');
            distance = data{1};
            time = data{2};

            % Append timestamp to LIDAR data with ID 1
            lidar(end+1,:) = [time, distance];
        end

        % Read the next line
        tline = fgetl(fid);
    end

    % Close the text file
    fclose(fid);
end

% Save data into separate .mat files
save('angular_velocity.mat', 'angular_velocity');
save('acceleration.mat', 'acceleration');
save('gp2.mat', 'gp2');
save('lidar.mat', 'lidar');


% Assign data to variables
% Normalize the timestamps for both datasets to start at zero
tdata_normal = datenum(acceleration(:, 1) - acceleration(1, 1)); % Assuming column 1 is time
timestamp_GP2 = datenum(gp2(:, 1) - gp2(1, 1));                   % Assuming column 1 is time

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
set(gcf, 'color', 'w', 'name', 'IMU displacement over time in xy direction ')
subplot(311)
plot(x1, x2, '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_1 (cm)')

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

% Load GP2 data
load('gp2.mat');  % Load GP2 data

% Extract timestamp and distance data from GP2
timestamp_gp2 = gp2(:, 1);
distance_gp2 = gp2(:, 2);

% Define a threshold for outliers
threshold = 3; % Typical value, but this can be adjusted
mean_GP2 = mean(gp2(:, 2));
std_GP2 = std(gp2(:, 2));

% Find indices of points that are outliers
outlier_indices = abs(gp2(:, 2) - mean_GP2) > (threshold * std_GP2);

% Remove outliers from both the distance and the timestamp data
gp2_cleaned = gp2(~outlier_indices, :);


figure;
plot(gp2_cleaned(:, 1), gp2_cleaned(:, 2), 'b.-', 'LineWidth', 1.5, 'MarkerSize', 10);
xlabel('Timestamp');
ylabel('Distance (units)'); % Update with the appropriate unit
title('Cleaned GP2 Sensor Data');
grid on;

% Define the window size for the moving average filter
window_size = 5;

% Apply the moving average filter
smoothed_distance_gp2 = movmean(gp2_cleaned(:, 2), window_size);

% Plot smoothed GP2 data against timestamps
figure;
plot(gp2_cleaned(:, 1), smoothed_distance_gp2, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 10);
xlabel('Timestamp (s)');
ylabel('Smoothed Distance (units)'); % Update with the appropriate unit
title('Smoothed Cleaned GP2 Sensor Data');
grid on;


% Interpolate GP2 data onto the normalized IMU timestamps
interpolated_smoothed_distance_GP2 = interp1(gp2_cleaned(:, 1), smoothed_distance_gp2, tdata_normal, 'pchip', 'extrap');

% Plot interpolated GP2 data
figure;
plot(tdata_normal, interpolated_smoothed_distance_GP2, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Interpolated Smoothed GP2 Distance (cm)');
title('Interpolated Smoothed GP2 Sensor Data');
grid on;

