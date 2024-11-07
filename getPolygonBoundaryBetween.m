function boundaryPoints = getPolygonBoundaryBetween(ray1, ray2, polygonVertices)
    % Finds points on the polygon boundary between two rays
    boundaryPoints = [];
    n = size(polygonVertices, 1) - 1;

    for i = 1:n
        edgeStart = polygonVertices(i, :);
        edgeEnd = polygonVertices(i+1, :);

        % Check if edgeStart is between ray1 and ray2
        if isPointInSector(edgeStart - ray1, ray1, ray2)
            boundaryPoints = [boundaryPoints; edgeStart];
        end
        % Check if the segment intersects the ray boundary
        intersection = lineSegmentIntersect(ray1, ray2, edgeStart, edgeEnd);
        if ~isempty(intersection)
            boundaryPoints = [boundaryPoints; intersection];
        end
    end
end

function inside = isPointInSector(point, ray1, ray2)
    % Checks if a point lies within the sector bounded by ray1 and ray2
    angle1 = atan2(ray1(2), ray1(1));
    angle2 = atan2(ray2(2), ray2(1));
    pointAngle = atan2(point(2), point(1));
    
    % Ensure the angles are within [0, 2*pi]
    angle1 = mod(angle1, 2*pi);
    angle2 = mod(angle2, 2*pi);
    pointAngle = mod(pointAngle, 2*pi);
    
    if angle1 < angle2
        inside = (pointAngle >= angle1) && (pointAngle <= angle2);
    else
        inside = (pointAngle >= angle1) || (pointAngle <= angle2);
    end
end
