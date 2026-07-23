function failureTable = createFailureResultRows(map,scenario,errorMessage,config)
%CREATEFAILURERESULTROWS Tao ba dong Success=false khi mot scenario loi.
methods = {'PIVOT_ONLY','FIXED_RADIUS','ADAPTIVE_PIVOT_OR_ARC'};
rows = repmat(emptyResultRow(),3,1);
for i = 1:3
    rows(i).MapName=map.name; rows(i).MapWidth=map.width;
    rows(i).MapHeight=map.height; rows(i).GridRows=size(map.occupancy,1);
    rows(i).GridColumns=size(map.occupancy,2);
    rows(i).TotalCells=numel(map.occupancy);
    rows(i).FreeCells=sum(~map.occupancy,'all');
    rows(i).ObstacleDensity=sum(map.occupancy,'all')/numel(map.occupancy);
    rows(i).ScenarioName=scenario.name;
    rows(i).AlgorithmRevision=config.algorithmRevision;
    rows(i).RobotProfile=config.robot.profileName;
    rows(i).RobotProfileMeasured=config.robot.measured;
    rows(i).Method=methods{i};
    rows(i).Planner='FAILED_BEFORE_PLANNING';
    rows(i).StartX=scenario.start(1); rows(i).StartY=scenario.start(2);
    rows(i).GoalX=scenario.goal(1); rows(i).GoalY=scenario.goal(2);
    rows(i).Success=false; rows(i).TaskCompletionRate=0;
    rows(i).ErrorMessage=char(errorMessage);
end
failureTable = struct2table(rows);
end
