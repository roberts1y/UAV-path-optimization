function intersection = lineSegmentIntersect(p1, p2, p3, p4)
    % Finds the intersection of two line segments p1-p2 and p3-p4 if it exists
    % Inputs:
    %   p1, p2 - Endpoints of the first line segment
    %   p3, p4 - Endpoints of the second line segment
    % Output:
    %   intersection - The point of intersection if it exists, otherwise empty

    % Initialize intersection as empty
    intersection = [];
    
    % Vector components
    s1 = p2 - p1;
    s2 = p4 - p3;
    
    % Calculate determinants
    denom = (-s2(1) * s1(2) + s1(1) * s2(2));
    if denom == 0
        % Lines are parallel, no intersection
        return;
    end

    s = (-s1(2) * (p1(1) - p3(1)) + s1(1) * (p1(2) - p3(2))) / denom;
    t = ( s2(1) * (p1(2) - p3(2)) - s2(2) * (p1(1) - p3(1))) / denom;
    
    % Check if s and t are within the range [0, 1], meaning the segments intersect
    if s >= 0 && s <= 1 && t >= 0 && t <= 1
        % Calculate the intersection point
        intersection = p1 + t * s1;
    end
end
