function map = initializeWarehouseMap(name, width, height, resolution, description)
%INITIALIZEWAREHOUSEMAP Tao ban do rong co tuong bao mot cell.
validateattributes(width, {'numeric'}, {'scalar','positive','finite'});
validateattributes(height, {'numeric'}, {'scalar','positive','finite'});
validateattributes(resolution, {'numeric'}, {'scalar','positive','finite'});

columns = round(width / resolution);
rows = round(height / resolution);
if abs(columns * resolution - width) > 1e-10 || ...
        abs(rows * resolution - height) > 1e-10
    error('Kich thuoc ban do phai la boi so cua resolution.');
end

occupancy = false(rows, columns);
occupancy([1 end], :) = true;
occupancy(:, [1 end]) = true;

map = struct('name', char(name), 'occupancy', occupancy, ...
    'width', width, 'height', height, 'resolution', resolution, ...
    'startGoalPairs', struct('name', {}, 'start', {}, 'goal', {}), ...
    'description', char(description));
end
