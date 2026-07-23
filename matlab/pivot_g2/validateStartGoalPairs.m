function validationTable = validateStartGoalPairs(maps, config)
%VALIDATESTARTGOALPAIRS Kiem tra vi tri, A* va footprint cho moi cap.
rows = repmat(struct('MapName','','ScenarioName','','StartValid',false, ...
    'GoalValid',false,'PathExists',false,'FootprintValid',false, ...
    'Message',''),0,1);
for k = 1:numel(maps)
    map = maps(k);
    planningOccupancy = inflateOccupancyGrid(map.occupancy, ...
        config.inflationRadius,map.resolution);
    for j = 1:numel(map.startGoalPairs)
        scenario = map.startGoalPairs(j);
        item = struct('MapName',map.name,'ScenarioName',scenario.name, ...
            'StartValid',false,'GoalValid',false,'PathExists',false, ...
            'FootprintValid',false,'Message','');
        [sr,sc,sv] = worldToGrid(scenario.start,map);
        [gr,gc,gv] = worldToGrid(scenario.goal,map);
        item.StartValid = sv && ~map.occupancy(sr,sc) && ~planningOccupancy(sr,sc);
        item.GoalValid = gv && ~map.occupancy(gr,gc) && ~planningOccupancy(gr,gc);
        if ~(item.StartValid && item.GoalValid)
            item.Message = 'Start/goal ngoai ban do, trong vat can hoac qua sat vat can.';
            rows(end+1,1) = item; %#ok<AGROW>
            continue;
        end
        [path,info] = planGridPath(planningOccupancy,scenario.start, ...
            scenario.goal,config,false);
        item.PathExists = info.success;
        if ~info.success
            item.Message = info.message;
            rows(end+1,1) = item; %#ok<AGROW>
            continue;
        end
        startHeading = atan2(path(2,2)-path(1,2),path(2,1)-path(1,1));
        goalHeading = atan2(path(end,2)-path(end-1,2), ...
            path(end,1)-path(end-1,1));
        startCollision = checkFootprintCollision([path(1,:) startHeading], ...
            map,config.robot,config.geometrySampleStep);
        goalCollision = checkFootprintCollision([path(end,:) goalHeading], ...
            map,config.robot,config.geometrySampleStep);
        startClearance = computeMinimumClearance([path(1,:) startHeading], ...
            map,config.robot);
        goalClearance = computeMinimumClearance([path(end,:) goalHeading], ...
            map,config.robot);
        item.FootprintValid = ~startCollision && ~goalCollision && ...
            startClearance >= config.robot.clearanceSafe-1e-9 && ...
            goalClearance >= config.robot.clearanceSafe-1e-9;
        if item.FootprintValid
            item.Message = 'OK';
        else
            item.Message = 'Footprint tai start hoac goal khong dat clearance.';
        end
        rows(end+1,1) = item; %#ok<AGROW>
    end
end
validationTable = struct2table(rows);
end
