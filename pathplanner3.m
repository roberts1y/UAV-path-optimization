% Define the launch point
launchPoint = [39.65, -87.75, 0];  % (latitude, longitude, altitude)

% Define the search area boundaries (polygon shape)
searchArea = [
    39.6, -87.7;
    39.55, -87.7;
    39.59, -87.5;
    39.6, -87.5
];

% Define UAV parameters
numUAVs = 4;              % Number of UAVs (equal to number of partitions)
maxSpeed = 10;            % Max speed (m/s)
UAVHeight = 25;           % UAV flight height (m)
sensorWidth = 20;         % Sensor coverage width (m)

% Split the search area polygon into partitions based on number of UAVs
subPolygons = splitPolygon2(searchArea, numUAVs, launchPoint(1:2));

% Set up the map and plot the search area
fig = figure;
g = geoaxes(fig, 'Basemap', 'satellite');
geolimits([launchPoint(1) - 0.01, launchPoint(1) + 0.01], [launchPoint(2) - 0.01, launchPoint(2) + 0.01]);
hold on;

% Plot the original search area boundary for reference
geoplot(g, [searchArea(:,1); searchArea(1,1)], [searchArea(:,2); searchArea(1,2)], 'cyan', 'LineWidth', 2);
scatter(g, launchPoint(2), launchPoint(1), 50, 'red', 'filled');  % Plot launch point

% Initialize arrays to store path metrics for each UAV
totalPathLengths = zeros(numUAVs, 1);
searchTimes = zeros(numUAVs, 1);

% Loop through each partition for path planning and visualization
for i = 1:numUAVs
    % Extract the vertices for this sub-polygon
    subPolygonVertices = subPolygons{i};
    
    % Plot each sub-polygon on the map
    geoplot(g, [subPolygonVertices(:,1); subPolygonVertices(1,1)], ...
            [subPolygonVertices(:,2); subPolygonVertices(1,2)], ...
            'black', 'LineWidth', 1.5, 'DisplayName', sprintf('Sub-Polygon %d', i));
    
    % Create a UAV coverage space for this partition
    uav_cs = uavCoverageSpace('Polygons', {subPolygonVertices}, ...
                              'UseLocalCoordinates', false, ...
                              'ReferenceLocation', launchPoint);
    uav_cs.UnitWidth = sensorWidth;   % Set sensor footprint width
    setCoveragePattern(uav_cs, 1, 'SweepAngle', 85);  % Define sweep angle for coverage

    % Initialize a coverage planner for the UAV
    cp = uavCoveragePlanner(uav_cs, 'Solver', 'MinTraversal');
    
    % Plan the UAV's coverage path from the launch point and plot it
    try
        [waypoints, solution] = plan(cp, launchPoint);  % Plan from the launch point
        geoplot(g, waypoints(:,1), waypoints(:,2), 'LineWidth', 1.5, 'DisplayName', sprintf('UAV Path %d', i));
        
        % Calculate path length and search time for this UAV
        totalPathLengths(i) = sum(vecnorm(diff(waypoints), 2, 2));  % Path length in meters
        searchTimes(i) = totalPathLengths(i) / maxSpeed;  % Time to complete the path in seconds
        
        % Display metrics for each UAV
        fprintf('UAV %d - Total Path Length: %.2f meters\n', i, totalPathLengths(i));
        fprintf('UAV %d - Estimated Search Time: %.2f seconds\n', i, searchTimes(i));
    catch exception
        fprintf('Error planning path for UAV %d: %s\n', i, exception.message);
    end
end

% Add a legend to the plot
legend(g, 'Search Area', 'Launch Point', 'Location', 'bestoutside');

% Display cumulative metrics for all UAVs
totalPathLengthAll = sum(totalPathLengths);
totalSearchTime = max(searchTimes);  % Longest individual search time determines total time
fprintf('Total Path Length for All UAVs: %.2f meters\n', totalPathLengthAll);
fprintf('Estimated Total Search Time: %.2f seconds\n', totalSearchTime);

hold off;
