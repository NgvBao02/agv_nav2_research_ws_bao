function maps = createMapSuite(config)
%CREATEMAPSUITE Tao bo sau ban do benchmark xac dinh va lap lai duoc.
if nargin < 1 || ~isfield(config, 'resolution')
    error('config.resolution la bat buoc.');
end
r = config.resolution;
maps = [createSmallWarehouseMap(r), ...
        createMediumWarehouseMap(r), ...
        createLargeWarehouseMap(r), ...
        createDenseRackWarehouseMap(r), ...
        createOpenFactoryMap(r), ...
        createMixedCorridorMap(r)];
for k = 1:numel(maps)
    [obstacleRows,obstacleColumns] = find(maps(k).occupancy);
    maps(k).obstacleRows = obstacleRows;
    maps(k).obstacleColumns = obstacleColumns;
    maps(k).obstacleCenters = gridToWorld(obstacleRows,obstacleColumns,r);
end
end
