launchPoint = [39.65, -87.75, 0];  % (latitude, longitude, altitude)

% define the search area boundaries
searchArea = [
    39.6, -87.7;
    39.55, -87.7;
    39.55, -87.5;
    39.6, -87.5
];  

% change based on choice of UAV
numUAVs = 4;              % number of UAVs
maxSpeed = 10;            % max speed (m/s)
UAVHeight = 25;           % UAV height (m)
sensorWidth = 20;         % sensor coverage width (m)

% initialize the coverage space object with bounded search area
cs = uavCoverageSpace('Polygons', {searchArea}, 'UseLocalCoordinates', false, 'ReferenceLocation', launchPoint);

% set additional UAV parameters
cs.UnitWidth = sensorWidth;      % sensor footprint width
ReferenceHeight = UAVHeight;     % UAV flight height

% visualize the search area on a map
fig = figure;
g = geoaxes(fig, 'Basemap', 'satellite');
geolimits([launchPoint(1) - 0.01, launchPoint(1) + 0.01], [launchPoint(2) - 0.01, launchPoint(2) + 0.01]);
show(cs, 'Parent', g);

% define specific sweep angles for optimal coverage
setCoveragePattern(cs, 1, 'SweepAngle', 85);

cp = uavCoveragePlanner(cs, 'Solver', 'Exhaustive');

% plan and visualize coverage path from launch point
[waypoints, solution] = plan(cp, launchPoint);
hold on;
geoplot(waypoints(:,1), waypoints(:,2), 'LineWidth', 1.5);  % Plot the path
geoplot(launchPoint(1), launchPoint(2), 'MarkerSize', 20, 'Marker', 'o', 'Color', 'r');  % Show launch point
legend("Path", "Launch Point");
hold off;

% display performance metrics
totalPathLength = sum(vecnorm(diff(waypoints), 2, 2));  % calculate path length
searchTime = totalPathLength / maxSpeed; 

fprintf('Total Path Length: %.2f meters\n', totalPathLength);
fprintf('Estimated Search Time: %.2f seconds\n', searchTime);