% Define the main polygon with the updated vertices
polygonVertices = [
    39.6, -88.7;    % Vertex 1 (top left)
    39.55, -87.75;  % Vertex 2 (bottom left)
    39.59, -87.5;   % Vertex 3 (bottom right)
    39.6, -87.3     % Vertex 4 (top right)
];

% Number of UAVs (equal to number of partitions)
numUAVs = 4;

% Define the launch point for UAVs, including altitude as the third component
launchPoint = [39.65, -87.75, 0];  % [Latitude, Longitude, Altitude]

% Partition the polygon into sub-regions using radial splits from the launch point
subPolygons = splitPolygonRadial(polygonVertices, numUAVs, launchPoint(1:2));  % Use 2D for split function

% UAV specifications
maxSpeed = 10;       % Max speed (m/s)
UAVHeight = 25;      % UAV flight height (m)
sensorWidth = 20;    % Sensor coverage width (m)

% Set up the map and plot the original polygon with adjusted geolimits
figure;
g = geoaxes('Basemap', 'satellite');

% Adjust geolimits to display both the launch point and polygon with ample space
latitudeRange = [min([polygonVertices(:,1); launchPoint(1)]) - 0.05, max([polygonVertices(:,1); launchPoint(1)]) + 0.05];
longitudeRange = [min([polygonVertices(:,2); launchPoint(2)]) - 0.1, max([polygonVertices(:,2); launchPoint(2)]) + 0.1];
geolimits(latitudeRange, longitudeRange);
hold on;

% Plot the main polygon boundary
geoplot([polygonVertices(:,1); polygonVertices(1,1)], ...
        [polygonVertices(:,2); polygonVertices(1,2)], ...
        'cyan', 'LineWidth', 2);

% Initialize arrays to store metrics for each UAV's path
totalPathLengths = zeros(numUAVs, 1);
searchTimes = zeros(numUAVs, 1);

% Loop through each partition for path planning and visualization
for i = 1:numUAVs
    % Extract the vertices for this sub-polygon
    subPolygonVertices = subPolygons{i};
    
    % Plot each sub-polygon on the map
    geoplot([subPolygonVertices(:,1); subPolygonVertices(1,1)], ...
            [subPolygonVertices(:,2); subPolygonVertices(1,2)], ...
            'black', 'LineWidth', 1.5);
    
    % Create a UAV coverage space for this partition
    uav_cs = uavCoverageSpace('Polygons', {subPolygonVertices}, 'UseLocalCoordinates', false, 'ReferenceLocation', launchPoint);  % Use 3D launch point
    uav_cs.UnitWidth = sensorWidth;  % Set sensor footprint width
    setCoveragePattern(uav_cs, 1, 'SweepAngle', 85);  % Define sweep angle for coverage

    % Initialize a coverage planner for the UAV
    cp = uavCoveragePlanner(uav_cs, 'Solver', 'MinTraversal');
    
    % Plan the UAV's coverage path from the launch point and plot it
    try
        [waypoints, solution] = plan(cp, launchPoint);  % Use the 3D launch point
        geoplot(waypoints(:,1), waypoints(:,2), 'LineWidth', 1.5);  % Plot the path
        
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

% Mark the launch point on the map
geoplot(launchPoint(1), launchPoint(2), 'MarkerSize', 20, 'Marker', 'o', 'Color', 'r');  % Plot the launch point
legend("UAV Paths", "Launch Point");

% Display cumulative metrics for all UAVs
totalPathLengthAll = sum(totalPathLengths);
totalSearchTime = max(searchTimes);  % Longest individual search time determines total time
fprintf('Total Path Length for All UAVs: %.2f meters\n', totalPathLengthAll);
fprintf('Estimated Total Search Time: %.2f seconds\n', totalSearchTime);

hold off;