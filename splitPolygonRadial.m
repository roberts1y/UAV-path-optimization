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