%Define project-specific inputs with altitude for ReferenceLocation
launhPoint = [39.65, -87.75, 0];  % UAV launch location (latitude, longitude, altitude)

% Define search area boundaries
searchArea = [
    39.6, -87.7;
    39.55, -87.7;
    39.55, -87.5;
    39.6, -87.5
];  % Boundary coordinates

% UAV parameters
numUAVs = 4;              % Number of UAVs
maxSpeed = 10;            % Max speed (m/s)
UAVHeight = 25;           % UAV height (m)
sensorWidth = 20;         % Sensor coverage width (m)

% Initialize the coverage space object with bounded search area
cs = uavCoverageSpace('Polygons', {searchArea}, 'UseLocalCoordinates', false, 'ReferenceLocation', launchPoint);

% Set additional UAV parameters
cs.UnitWidth = sensorWidth;      % Sensor footprint width
ReferenceHeight = UAVHeight;     % UAV flight height

% Visualize the search area on a map
fig = figure;
g = geoaxes(fig, 'Basemap', 'satellite');
geolimits([launchPoint(1) - 0.01, launchPoint(1) + 0.01], [launchPoint(2) - 0.01, launchPoint(2) + 0.01]);
show(cs, 'Parent', g);

% Define specific sweep angles for optimal coverage
setCoveragePattern(cs, 1, 'SweepAngle', 85);  % Adjust for road alignment if needed

% Initialize UAV coverage planner with exhaustive path planning
cp = uavCoveragePlanner(cs, 'Solver', 'Exhaustive');

% Plan and visualize coverage path from launch point
[waypoints, solution] = plan(cp, launchPoint);
hold on;
geoplot(waypoints(:,1), waypoints(:,2), 'LineWidth', 1.5);  % Plot the path
geoplot(launchPoint(1), launchPoint(2), 'MarkerSize', 20, 'Marker', 'o', 'Color', 'r');  % Show launch point
legend("Path", "Launch Point");
hold off;

% Display performance metrics
totalPathLength = sum(vecnorm(diff(waypoints), 2, 2));  % Calculate path length
searchTime = totalPathLength / maxSpeed;  % Estimate search time based on max speed

fprintf('Total Path Length: %.2f meters\n', totalPathLength);
fprintf('Estimated Search Time: %.2f seconds\n', searchTime);


