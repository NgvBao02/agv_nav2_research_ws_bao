function occupancy = addRectangleObstacle(occupancy, resolution, rectangle)
%ADDRECTANGLEOBSTACLE Them vat can [x y width height] vao occupancy grid.
validateattributes(rectangle, {'numeric'}, {'vector','numel',4,'finite'});
rectangle = double(rectangle(:).');
if rectangle(3) <= 0 || rectangle(4) <= 0
    error('Chieu rong va chieu cao vat can phai duong.');
end

[rows, columns] = size(occupancy);
x0 = rectangle(1); y0 = rectangle(2);
x1 = x0 + rectangle(3); y1 = y0 + rectangle(4);
columnCenters = ((1:columns) - 0.5) * resolution;
rowCenters = ((1:rows) - 0.5) * resolution;
columnMask = columnCenters >= x0 & columnCenters <= x1;
rowMask = rowCenters >= y0 & rowCenters <= y1;
occupancy(rowMask, columnMask) = true;
end
