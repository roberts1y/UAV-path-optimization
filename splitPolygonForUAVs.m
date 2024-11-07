function partitions = splitPolygonForUAVs(searchArea, numUAVs)
    % Ensure the search area is a valid polygon with at least three points
    if size(searchArea, 1) < 3
        error('Search area must be a polygon with at least three corners.');
    end
    
    % Calculate the centroid of the search area
    centroid = mean(searchArea, 1);
    
    % Calculate the angle increment for each UAV partition
    angleIncrement = 2 * pi / numUAVs;
    
    % Initialize partitions cell array
    partitions = cell(numUAVs, 1);
    
    % Loop to create each polygon partition
    for i = 1:numUAVs
        % Define the start and end angle for the current partition
        startAngle = (i - 1) * angleIncrement;
        endAngle = i * angleIncrement;
        
        % Initialize a partition with the centroid as the starting point
        partition = centroid;
        
        % Loop through each edge of the original polygon
        for j = 1:size(searchArea, 1)
            % Get the current edge vertices
            v1 = searchArea(j, :);
            v2 = searchArea(mod(j, size(searchArea, 1)) + 1, :);
            
            % Calculate angles for vertices relative to centroid
            angle1 = atan2(v1(2) - centroid(2), v1(1) - centroid(1));
            angle2 = atan2(v2(2) - centroid(2), v2(1) - centroid(1));
            
            % Normalize angles to [0, 2*pi]
            if angle1 < 0, angle1 = angle1 + 2 * pi; end
            if angle2 < 0, angle2 = angle2 + 2 * pi; end
            
            % Check if the edge intersects with the partition's angular range
            if (startAngle <= angle1 && angle1 < endAngle) || ...
               (startAngle <= angle2 && angle2 < endAngle) || ...
               (angle1 < startAngle && angle2 > endAngle)
                % Add the intersection points to the partition
                partition = [partition; v1; v2];
            end
        end
        
        % Remove duplicate points and close the polygon
        partition = removeDuplicatePoints(partition);
        
        % Store the cleaned partition
        partitions{i} = partition;
    end
end

% Helper function to remove duplicate points and close the polygon if needed
function cleanPartition = removeDuplicatePoints(partition)
    % Remove duplicate points
    [~, uniqueIdx] = unique(partition, 'rows', 'stable');
    partition = partition(uniqueIdx, :);
    
    % Ensure the polygon is closed by adding the starting point at the end, if needed
    if ~isequal(partition(1, :), partition(end, :))
        partition = [partition; partition(1, :)];
    end
    
    cleanPartition = partition;
end
