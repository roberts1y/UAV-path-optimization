function subPolygons = splitPolygon2(polygonVertices, numPartitions, startPoint)
    % splitPolygon - Split a polygon into radial partitions from a given starting point
    % with approximately equal areas.
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

    % Calculate evenly spaced angles for dividing the area into radial sections
    angles = linspace(0, 2 * pi, numPartitions + 1);

    % Initialize output cell array for sub-polygons
    subPolygons = cell(numPartitions, 1);

    % Loop through each partition to create radial sub-polygons
    for i = 1:numPartitions
        % Define the angle range for this partition
        angle1 = angles(i);
        angle2 = angles(i + 1);

        % Extend rays from the start point in the directions of angle1 and angle2
        ray1 = extendRayToPolygon(startPoint, angle1, polygonVertices);
        ray2 = extendRayToPolygon(startPoint, angle2, polygonVertices);

        % Ensure rays intersect with the polygon boundary; approximate if necessary
        if isempty(ray1), ray1 = startPoint + [cos(angle1), sin(angle1)] * 1e-3; end
        if isempty(ray2), ray2 = startPoint + [cos(angle2), sin(angle2)] * 1e-3; end

        % Get boundary points between ray1 and ray2 on the polygon
        boundaryPoints = getPolygonBoundaryBetween(ray1, ray2, polygonVertices);

        % Construct the sub-polygon with the start point, ray1, boundary points, and ray2
        subPolygonVertices = [startPoint; ray1; boundaryPoints; ray2; startPoint];

        % Remove duplicate vertices and check for validity
        subPolygonVertices = unique(subPolygonVertices, 'rows', 'stable');
        if size(subPolygonVertices, 1) >= 3
            subPolygons{i} = subPolygonVertices;
        else
            subPolygons{i} = [];
        end
    end

    % Remove any empty cells caused by degenerate polygons
    subPolygons = subPolygons(~cellfun('isempty', subPolygons));
end

function pointOnBoundary = extendRayToPolygon(startPoint, angle, polygonVertices)
    % Extends a ray from the start point at a given angle until it intersects the polygon boundary
    maxDistance = 1e6;  % Large distance to extend the ray
    rayEndPoint = startPoint + maxDistance * [cos(angle), sin(angle)];
    
    % Find the intersection point with the polygon boundary
    pointOnBoundary = [];
    minDistance = inf;  % Track the closest intersection point
    for i = 1:size(polygonVertices, 1) - 1
        edgeStart = polygonVertices(i, :);
        edgeEnd = polygonVertices(i + 1, :);
        intersection = lineSegmentIntersect(startPoint, rayEndPoint, edgeStart, edgeEnd);
        if ~isempty(intersection)
            % Only consider the nearest intersection
            distance = norm(intersection - startPoint);
            if distance < minDistance
                minDistance = distance;
                pointOnBoundary = intersection;
            end
        end
    end
end

function boundaryPoints = getPolygonBoundaryBetween(ray1, ray2, polygonVertices)
    % Finds points on the polygon boundary between the two rays
    boundaryPoints = [];
    n = size(polygonVertices, 1) - 1;

    for i = 1:n
        edgeStart = polygonVertices(i, :);
        edgeEnd = polygonVertices(i + 1, :);

        % Check if edgeStart lies between ray1 and ray2
        if isPointInSector(edgeStart - ray1, ray1, ray2)
            boundaryPoints = [boundaryPoints; edgeStart];
        end

        % Check if the edge intersects the boundary of the rays
        intersection = lineSegmentIntersect(ray1, ray2, edgeStart, edgeEnd);
        if ~isempty(intersection)
            boundaryPoints = [boundaryPoints; intersection];
        end
    end
end

function inside = isPointInSector(point, ray1, ray2)
    % Checks if a point is within the angular sector between ray1 and ray2
    angle1 = atan2(ray1(2), ray1(1));
    angle2 = atan2(ray2(2), ray2(1));
    pointAngle = atan2(point(2), point(1));

    angle1 = mod(angle1, 2 * pi);
    angle2 = mod(angle2, 2 * pi);
    pointAngle = mod(pointAngle, 2 * pi);

    if angle1 < angle2
        inside = (pointAngle >= angle1) && (pointAngle <= angle2);
    else
        inside = (pointAngle >= angle1) || (pointAngle <= angle2);
    end
end

function intersection = lineSegmentIntersect(p1, p2, p3, p4)
    % Determines the intersection point between line segments p1-p2 and p3-p4
    intersection = [];
    s1 = p2 - p1;
    s2 = p4 - p3;
    denom = (-s2(1) * s1(2) + s1(1) * s2(2));
    if denom == 0
        return; % Parallel lines
    end

    s = (-s1(2) * (p1(1) - p3(1)) + s1(1) * (p1(2) - p3(2))) / denom;
    t = ( s2(1) * (p1(2) - p3(2)) - s2(2) * (p1(1) - p3(1))) / denom;

    if s >= 0 && s <= 1 && t >= 0 && t <= 1
        intersection = p1 + t * s1;
    end
end
