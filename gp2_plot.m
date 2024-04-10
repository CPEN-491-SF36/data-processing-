clear all
close all
clc

% Define file names
file_names = {'gp2_13.txt', 'lidar_13.txt'};

% Initialize arrays to store data
gp2_data = cell(1, 4); % Cell array to store data for each GP2 ID
lidar_data = cell(1, 4); % Cell array to store data for each LIDAR ID

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
       if contains(tline, 'GP2_ID:')
            % Extract GP2 sensor data
            data = textscan(tline, 'GP2_ID: %d Distance: %f Time: %f');
            id = data{1};
            distance = data{2};
            time = data{3};

            % Filter out zero distance values
            if distance ~= 0
                % Append data to the respective GP2 ID cell array
                gp2_data{id} = [gp2_data{id}; time, distance];
            end
        elseif contains(tline, 'LIDAR_ID:')
            % Extract LIDAR sensor data
            data = textscan(tline, 'LIDAR_ID: %d Distance: %f Time: %f');
            id = data{1};
            distance = data{2};
            time = data{3};

            % Append data to the respective LIDAR ID cell array
            lidar_data{id} = [lidar_data{id}; time, distance];
        end

        % Read the next line
        tline = fgetl(fid);
    end

    % Close the text file
    fclose(fid);
end

% Remove great outlier from Lidar ID 4 data
lidar_id4 = lidar_data{4};
if ~isempty(lidar_id4)
    % Define function to remove great outliers using manual threshold
    remove_great_outliers = @(data) data(data(:,2) <= 35, :); % Adjust threshold as per your data
    
    % Remove great outliers for Lidar ID 4 data
    cleaned_lidar_id4 = remove_great_outliers(lidar_id4);
    
    % Plotting data for GP2 sensors
    for id = 1:4
        if ~isempty(gp2_data{id})
            % Remove zero distance values from GP2 data
            cleaned_gp2_data = gp2_data{id}(gp2_data{id}(:,2) ~= 0, :);
            figure;
            plot(cleaned_gp2_data(:,1), cleaned_gp2_data(:,2), 'b'); % Blue color for GP2 data
            title(['Cleaned GP2 Sensor ID ' num2str(id)]);
            xlabel('Time');
            ylabel('Distance');
            hold on; % Hold the plot for adding Lidar data
        end
    end
    
    % Plot cleaned Lidar ID 4 data
    if ~isempty(cleaned_lidar_id4)
        plot(cleaned_lidar_id4(:,1), cleaned_lidar_id4(:,2), 'r'); % Red color for Lidar data
        title('1D Distance Test - GP2 IR and Garmin LIDAR Sensors');
        xlabel('Time[s]');
        ylabel('Distance[cm]');
        legend('GP2', 'Lidar', 'Location', 'southoutside', 'Orientation', 'horizontal');
        grid on; 
    end
end
