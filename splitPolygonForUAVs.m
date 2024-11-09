function partitions = splitPolygonForUAVs(polygonCoords, launchPoint, numUAVs)
    % Splits a polygon into `numUAVs` equal radial partitions from the launch point.
    %
    % Args:
    %     polygonCoords (Nx2 array): Array of [latitude, longitude] coordinates for the polygon.
    %     launchPoint (1x2 array): [latitude, longitude] of the launch point.
    %     numUAVs (int): Number of UAVs (up to 6).
    %
    % Returns:
    %     partitions: Cell array of [latitude, longitude] arrays, each representing a partition polygon.

    % Create the main search area polygon using polyshape
    mainPolygon = polyshape(polygonCoords(:,1), polygonCoords(:,2));
    
    % Initialize a cell array to hold each partitioned sub-polygon
    partitions = cell(1, numUAVs);
    
    % Calculate the angle increment for each sector
    angleIncrement = 360 / numUAVs;
    
    % Loop over each UAV to create its sector
    for i = 1:numUAVs
        % Calculate the start and end angles for the current sector
        angleStart = (i - 1) * angleIncrement;
        angleEnd = i * angleIncrement;
        
        % Calculate the boundary points for the sector lines based on the angle and distance
        distance = 0.01;  % Distance in degrees, adjust as needed for coverage
        
        % Compute the boundary points for the sector
        startLat = launchPoint(1) + cosd(angleStart) * distance;
        startLon = launchPoint(2) + sind(angleStart) * distance;
        
        endLat = launchPoint(1) + cosd(angleEnd) * distance;
        endLon = launchPoint(2) + sind(angleEnd) * distance;
        
        % Create the sector as a triangle from launch point to boundary points
        sectorPolygon = polyshape([launchPoint(1), startLat, endLat], ...
                                  [launchPoint(2), startLon, endLon]);
        
        % Intersect the sector with the main polygon to get the bounded area
        partition = intersect(mainPolygon, sectorPolygon);
        
        % Store the partition's vertices as a matrix
        [lat, lon] = boundary(partition);
        partitions{i} = [lat, lon];
    end
end

% Example usage
polygonCoords = [39.6, -87.7; 39.55, -87.7; 39.59, -87.5; 39.6, -87.5];
launchPoint = [39.65, -87.75];
numUAVs = 4;

% Get partitions
partitions = splitPolygonForUAVs(polygonCoords, launchPoint, numUAVs);

% Display partitions
for i = 1:numUAVs
    disp(['UAV ', num2str(i), ' Partition Coordinates:']);
    disp(partitions{i});
end
