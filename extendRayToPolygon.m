function pointOnBoundary = extendRayToPolygon(startPoint, angle, polygonVertices)
    % Extends a ray from the start point at a given angle until it intersects the polygon boundary
    maxDistance = 1e6;  % Large distance to ensure the ray extends far enough
    rayEndPoint = startPoint + maxDistance * [cos(angle), sin(angle)];
    
    % Find the intersection point with the polygon boundary
    pointOnBoundary = [];
    minDistance = inf;  % Initialize to find the closest intersection
    for i = 1:size(polygonVertices, 1) - 1
        edgeStart = polygonVertices(i, :);
        edgeEnd = polygonVertices(i+1, :);
        intersection = lineSegmentIntersect(startPoint, rayEndPoint, edgeStart, edgeEnd);
        if ~isempty(intersection)
            % Check if this intersection is the closest to startPoint
            distance = norm(intersection - startPoint);
            if distance < minDistance
                minDistance = distance;
                pointOnBoundary = intersection;
            end
        end
    end
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
