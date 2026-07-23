function inflated = inflateOccupancyGrid(occupancy, radius, resolution)
%INFLATEOCCUPANCYGRID No vat can bang kernel tron, khong dung toolbox.
validateattributes(radius, {'numeric'}, {'scalar','nonnegative','finite'});
cellRadius = ceil(radius / resolution);
if cellRadius == 0
    inflated = logical(occupancy);
    return;
end
[dx, dy] = meshgrid(-cellRadius:cellRadius, -cellRadius:cellRadius);
kernel = hypot(dx, dy) <= radius / resolution + 0.5;
inflated = conv2(double(occupancy), double(kernel), 'same') > 0;
end
