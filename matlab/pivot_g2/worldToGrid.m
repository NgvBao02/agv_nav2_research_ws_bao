function [row, column, valid] = worldToGrid(point, map)
%WORLDTOGRID Doi toa do met sang chi so [row, column] cua cell.
point = double(point);
column = floor(point(:,1) ./ map.resolution) + 1;
row = floor(point(:,2) ./ map.resolution) + 1;
valid = row >= 1 & row <= size(map.occupancy,1) & ...
        column >= 1 & column <= size(map.occupancy,2);
end
