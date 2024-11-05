function subPolygons = splitPolygonRadial(polygonVertices, numPartitions, startPoint)
    % splitPolygonRadial - Split a polygon into radial partitions from a given starting point
    %
    % Inputs:
    %   polygonVertices - Nx2 matrix of [latitude, longitude] defining the polygon vertices
    %   numPartitions - Number of partitions to split the polygon into
    %   startPoint - 1x2 array [latitude, longitude] of the starting point
    %
    % Output:
    %   subPolygons - Cell array containing vertices of each radial sub-polygon

    % Ensure polygon is closed (add first point to end if necessary)
    if ~isequal(polygonVertices(1,:), polygonVertices(end,:))
        polygonVertices(end+1, :) = polygonVertices(1, :);
    end

    % Calculate angles for dividing the area into radial sections
    angles = linspace(0, 2*pi, numPartitions + 1);

    % Initialize output cell array for sub-polygons
    subPolygons = cell(numPartitions, 1);

    % Loop through each partition and create radial sub-polygons
    for i = 1:numPartitions
        % Define the angle range for this partition
        angle1 = angles(i);
        angle2 = angles(i+1);

        % Create lines extending from the start point to the polygon boundary
        ray1 = extendRayToPolygon(startPoint, angle1, polygonVertices);
        ray2 = extendRayToPolygon(startPoint, angle2, polygonVertices);

        % If no intersection is found, use an approximate point far from startPoint
        if isempty(ray1), ray1 = startPoint + [cos(angle1), sin(angle1)] * 1e-3; end
        if isempty(ray2), ray2 = startPoint + [cos(angle2), sin(angle2)] * 1e-3; end

        % Construct the sub-polygon by connecting startPoint, ray1, polygon edge, and ray2
        boundaryPoints = getPolygonBoundaryBetween(ray1, ray2, polygonVertices);
        subPolygonVertices = [startPoint; ray1; boundaryPoints; ray2];

        % Store the vertices of this sub-polygon in the output array
        subPolygons{i} = subPolygonVertices;
    end
end

function boundaryPoints = getPolygonBoundaryBetween(ray1, ray2, polygonVertices)
    % Finds points on the polygon boundary between the two rays
    boundaryPoints = [];
    n = size(polygonVertices, 1) - 1;

    for i = 1:n
        edgeStart = polygonVertices(i, :);
        edgeEnd = polygonVertices(i+1, :);
        
        % Check if edgeStart is between ray1 and ray2
        if isPointInSector(edgeStart, ray1, ray2)
            boundaryPoints = [boundaryPoints; edgeStart];
        end
    end
end

function pointOnBoundary = extendRayToPolygon(startPoint, angle, polygonVertices)
    % Extends a ray from the start point at a given angle until it intersects the polygon boundary
    maxDistance = 1e6;  % Large distance for extension
    rayEndPoint = startPoint + maxDistance * [cos(angle), sin(angle)];
    
    % Find the intersection point with the polygon boundary
    pointOnBoundary = [];
    for i = 1:size(polygonVertices, 1) - 1
        edgeStart = polygonVertices(i, :);
        edgeEnd = polygonVertices(i+1, :);
        intersection = lineSegmentIntersect(startPoint, rayEndPoint, edgeStart, edgeEnd);
        if ~isempty(intersection)
            pointOnBoundary = intersection;
            break;
        end
    end
end

function inside = isPointInSector(point, ray1, ray2)
    % Checks if a point lies within the sector bounded by ray1 and ray2 relative to the origin (startPoint)
    if isempty(ray1) || isempty(ray2)
        inside = false;
        return;
    end
    angle1 = atan2(ray1(2), ray1(1));
    angle2 = atan2(ray2(2), ray2(1));
    pointAngle = atan2(point(2), point(1));
    inside = pointAngle >= min(angle1, angle2) && pointAngle <= max(angle1, angle2);
end

function intersection = lineSegmentIntersect(p1, p2, p3, p4)
    % Finds the intersection of two line segments p1-p2 and p3-p4 if it exists
    intersection = [];
    s1 = p2 - p1;
    s2 = p4 - p3;
    s = (-s1(2) * (p1(1) - p3(1)) + s1(1) * (p1(2) - p3(2))) / (-s2(1) * s1(2) + s1(1) * s2(2));
    t = ( s2(1) * (p1(2) - p3(2)) - s2(2) * (p1(1) - p3(1))) / (-s2(1) * s1(2) + s1(1) * s2(2));
    if s >= 0 && s <= 1 && t >= 0 && t <= 1
        intersection = p1 + t * s1;
    end
end