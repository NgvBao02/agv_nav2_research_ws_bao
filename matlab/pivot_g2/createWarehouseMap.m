function map = createWarehouseMap(config)
%CREATEWAREHOUSEMAP Tuong thich voi mo phong cu: tra ve SMALL_WAREHOUSE.
if nargin < 1
    config = defaultCornerOptimizerConfig();
end
map = createSmallWarehouseMap(config.resolution);
end
