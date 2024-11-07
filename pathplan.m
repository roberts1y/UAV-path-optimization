% Define launch point (latitude, longitude, altitude)
launchPoint = [39.65, -87.75, 0];  % (latitude, longitude, altitude)

% Define the search area boundaries
searchArea = [
    39.6, -87.7;
    39.55, -87.7;
    39.59, -87.5;
    39.6, -87.5
];

% Number of UAVs and other parameters
numUAVs = 4;              % Number of UAVs (adjustable up to 6)
maxSpeed = 10;            % Max speed (m/s)
UAVHeight = 25;           % UAV flight height (m)
sensorWidth = 20;         % Sensor coverage width (m)

% Partition the search area for each UAV
partitions = splitPolygonForUAVs(searchArea, numUAVs);

% Set up figure and map for visualization
fig = figure;
g = geoaxes(fig, 'Basemap', 'satellite');
geolimits([launchPoint(1) - 0.01, launchPoint(1) + 0.01], ...
    [launchPoint(2) - 0.01, launchPoint(2) + 0.01]);

% Define colors for plotting each UAV's path and partition distinctly (up to 6 UAVs)
colors = ['b', 'g', 'm', 'c', 'y', 'k'];  % Support up to 6 different colors

% Initialize legend entries
legendEntries = strings(1, numUAVs);

% Plan and visualize coverage path for each UAV within its partition
for i = 1:numUAVs
    % Extract the partition polygon for the current UAV
    partitionArea = partitions{i};
    
    % Check if the partition has at least three points
    if size(partitionArea, 1) < 3
        warning('UAV %d partition has fewer than three points. Skipping this partition.', i);
        continue;  % Skip this iteration if the partition is not a valid polygon
    end
    
    % Plot the partition polygon to show the area for each UAV
    geoplot(g, [partitionArea(:,1); partitionArea(1,1)], ...
        [partitionArea(:,2); partitionArea(1,2)], 'LineWidth', 1.5, ...
        'Color', colors(i), 'DisplayName', sprintf("UAV %d Area", i));
    hold on;
    
    % Define the coverage space and planner for each partition
    cs = uavCoverageSpace('Polygons', {partitionArea}, ...
        'UseLocalCoordinates', false, 'ReferenceLocation', launchPoint);
    cs.UnitWidth = sensorWidth;
    
    % Plan the path within this partition for the current UAV
    cp = uavCoveragePlanner(cs, 'Solver', 'MinTraversal');
    [waypoints, solution] = plan(cp, launchPoint);
    
    % Plot the path for the current UAV
    geoplot(g, waypoints(:,1), waypoints(:,2), 'LineWidth', 1.5, ...
        'Color', colors(i), 'DisplayName', sprintf("UAV %d Path", i));
    
    % Store legend entry
    legendEntries(i) = sprintf("UAV %d Path", i);
end

% Plot the launch point
geoplot(g, launchPoint(1), launchPoint(2), 'MarkerSize', 20, ...
    'Marker', 'o', 'Color', 'r', 'DisplayName', 'Launch Point');

% Add the legend for all UAVs
legend([legendEntries, "Launch Point"]);
hold off;

% Display performance metrics for each UAV
for i = 1:numUAVs
    totalPathLength = sum(vecnorm(diff(waypoints), 2, 2));  % Calculate path length
    searchTime = totalPathLength / maxSpeed; 
    fprintf('UAV %d Path Length: %.2f meters\n', i, totalPathLength);
    fprintf('UAV %d Estimated Search Time: %.2f seconds\n', i, searchTime);
end
