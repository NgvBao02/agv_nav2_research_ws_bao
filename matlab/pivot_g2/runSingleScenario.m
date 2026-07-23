function scenarioResult = runSingleScenario(map, scenario, config)
%RUNSINGLESCENARIO Chay cung A* va ba cach thuc hien goc cua.
if ~isfield(map,'obstacleCenters')
    [map.obstacleRows,map.obstacleColumns] = find(map.occupancy);
    map.obstacleCenters = gridToWorld(map.obstacleRows,map.obstacleColumns, ...
        map.resolution);
end
repeatCount = max(1,round(config.numRepeats));
planningOccupancy = inflateOccupancyGrid(map.occupancy, ...
    config.inflationRadius,map.resolution);

planningTimes = zeros(repeatCount,1);
rawPath = zeros(0,2);
info = struct();
for r = 1:repeatCount
    timer = tic;
    captureTrace = config.capturePlannerTrace && r==1;
    [pathHere,infoHere] = planGridPath(planningOccupancy,scenario.start, ...
        scenario.goal,config,captureTrace);
    planningTimes(r) = toc(timer);
    if r == 1
        rawPath = pathHere;
        info = infoHere;
    end
    if ~infoHere.success
        error('AStar:NoPath','%s / %s: %s',map.name,scenario.name,infoHere.message);
    end
end
if size(rawPath,1) < 2
    error('AStar:InvalidPath','Duong A* khong du hai diem.');
end

% Kiem tra footprint start-goal theo huong cua duong da lap.
startHeading = atan2(rawPath(2,2)-rawPath(1,2),rawPath(2,1)-rawPath(1,1));
goalHeading = atan2(rawPath(end,2)-rawPath(end-1,2), ...
    rawPath(end,1)-rawPath(end-1,1));
for pose = {[rawPath(1,:) startHeading],[rawPath(end,:) goalHeading]}
    if checkFootprintCollision(pose{1},map,config.robot,config.geometrySampleStep) || ...
            computeMinimumClearance(pose{1},map,config.robot) < ...
            config.robot.clearanceSafe-1e-9
        error('Validation:UnsafeEndpoint', ...
            'Footprint tai start hoac goal khong an toan.');
    end
end

cornerDetectionTimes = zeros(repeatCount,1);
for r = 1:repeatCount
    timer = tic;
    reducedHere = removeCollinearPoints(rawPath);
    cornersHere = detectCorners(reducedHere);
    cornerDetectionTimes(r) = toc(timer);
    if r == 1
        reducedPath = reducedHere;
        corners = cornersHere;
    end
end

methods = {'PIVOT_ONLY','FIXED_RADIUS','ADAPTIVE_PIVOT_OR_ARC'};
methodTemplate = struct('name','','decisions',struct([]),'reference',struct(), ...
    'simulation',struct(),'metrics',struct(),'timing',struct());
methodResults = repmat(methodTemplate,numel(methods),1);
resultRows = repmat(emptyResultRow(),numel(methods),1);

for m = 1:numel(methods)
    optimizationTimes = zeros(repeatCount,1);
    footprintTimes = zeros(repeatCount,1);
    selectionTimes = zeros(repeatCount,1);
    referenceTimes = zeros(repeatCount,1);
    simulationTimes = zeros(repeatCount,1);
    totalTimes = zeros(repeatCount,1);
    for r = 1:repeatCount
        [decisionsHere,optimizerTiming] = chooseManeuversForMethod( ...
            corners,map,config,methods{m});
        referenceTimer = tic;
        referenceHere = buildReferenceTrajectory(reducedPath,decisionsHere,config);
        referenceHere = generateSpeedProfile(referenceHere,config);
        referenceTimes(r) = toc(referenceTimer);
        simulationTimer = tic;
        simulationHere = simulateDifferentialDrive(referenceHere,map,config);
        simulationTimes(r) = toc(simulationTimer);
        optimizationTimes(r) = optimizerTiming.cornerOptimizationTime;
        footprintTimes(r) = optimizerTiming.footprintCheckTime;
        selectionTimes(r) = optimizerTiming.maneuverSelectionTime;
        totalTimes(r) = planningTimes(r)+cornerDetectionTimes(r)+ ...
            optimizationTimes(r)+referenceTimes(r)+simulationTimes(r);
        if r == 1
            decisions = decisionsHere;
            reference = referenceHere;
            simulation = simulationHere;
            firstOptimizerTiming = optimizerTiming;
        end
    end
    metrics = computeTrackingMetrics(simulation,reference,decisions,config);
    timing = struct('planningSamples',planningTimes, ...
        'cornerDetectionSamples',cornerDetectionTimes, ...
        'cornerOptimizationSamples',optimizationTimes, ...
        'footprintCheckSamples',footprintTimes, ...
        'maneuverSelectionSamples',selectionTimes, ...
        'referenceBuildSamples',referenceTimes, ...
        'simulationSamples',simulationTimes,'totalSamples',totalTimes);
    methodResults(m) = struct('name',methods{m},'decisions',decisions, ...
        'reference',reference,'simulation',simulation,'metrics',metrics, ...
        'timing',timing);
    resultRows(m) = buildRow(methods{m},metrics,firstOptimizerTiming, ...
        timing,map,scenario,rawPath,reducedPath,corners,info,config);
end

scenarioResult = struct('map',map,'scenario',scenario, ...
    'planningOccupancy',planningOccupancy,'rawPath',rawPath, ...
    'reducedPath',reducedPath,'corners',corners,'astarInfo',info, ...
    'methodResults',methodResults,'resultTable',struct2table(resultRows));
end

function row = buildRow(method,metrics,optimizerTiming,timing,map,scenario, ...
        rawPath,reducedPath,corners,astarInfo,config)
row = emptyResultRow();
row.MapName=map.name; row.MapWidth=map.width; row.MapHeight=map.height;
row.GridRows=size(map.occupancy,1); row.GridColumns=size(map.occupancy,2);
row.TotalCells=numel(map.occupancy); row.FreeCells=sum(~map.occupancy,'all');
row.ObstacleDensity=sum(map.occupancy,'all')/numel(map.occupancy);
row.ScenarioName=scenario.name;
row.AlgorithmRevision=config.algorithmRevision;
row.RobotProfile=config.robot.profileName;
row.RobotProfileMeasured=config.robot.measured;
row.Method=method;
row.Planner=astarInfo.planner; row.AStarTurnPenalty=astarInfo.turnPenalty;
row.AStarTurnCostRatio=astarInfo.turnCostRatio;
row.AStarExpandedNodes=astarInfo.expandedNodes;
row.AStarPathCost=astarInfo.pathCostCells;
row.StartX=scenario.start(1); row.StartY=scenario.start(2);
row.GoalX=scenario.goal(1); row.GoalY=scenario.goal(2);
row.AStarPlanningTime=mean(timing.planningSamples);
row.AStarPlanningTimeStd=std(timing.planningSamples);
row.AStarPlanningTimeMin=min(timing.planningSamples);
row.AStarPlanningTimeMax=max(timing.planningSamples);
row.CornerDetectionTime=mean(timing.cornerDetectionSamples);
row.FootprintCheckTime=mean(timing.footprintCheckSamples);
row.ManeuverSelectionTime=mean(timing.maneuverSelectionSamples);
row.CornerOptimizationTime=mean(timing.cornerOptimizationSamples);
row.CornerOptimizationTimeStd=std(timing.cornerOptimizationSamples);
row.SimulationComputationTime=mean(timing.simulationSamples);
row.TotalAlgorithmTime=mean(timing.totalSamples);
row.TotalAlgorithmTimeStd=std(timing.totalSamples);
row.TotalAlgorithmTimeMin=min(timing.totalSamples);
row.TotalAlgorithmTimeMax=max(timing.totalSamples);
row.NumberOfRawWaypoints=size(rawPath,1);
row.NumberOfReducedWaypoints=size(reducedPath,1);
row.NumberOfCorners=numel(corners);
row.NumberOfArcCandidates=optimizerTiming.numberOfArcCandidates;
row.NumberOfArcCorners=metrics.NumberOfArcCorners;
row.NumberOfPivotCorners=metrics.NumberOfPivotCorners;
row.NumberOfRejectedArcCandidates=optimizerTiming.numberOfRejectedArcCandidates;
metricNames = {'ReferencePathLength','ActualPathLength','CompletionTime', ...
    'NumberOfFullStops','TotalStoppedTime','TotalPivotAngle', ...
    'AverageLinearVelocity','PositionRMSE','HeadingRMSE','MinimumClearance', ...
    'Jv','Jomega','MaximumLeftWheelVelocity','MaximumRightWheelVelocity', ...
    'LimitViolationCount','CollisionCount','TimePerMeter','StopsPerCorner', ...
    'JvPerMeter','JomegaPerMeter','ArcSelectionRate','TaskCompletionRate','Success'};
for i = 1:numel(metricNames)
    row.(metricNames{i}) = metrics.(metricNames{i});
end
row.OptimizationTimePerCorner = row.CornerOptimizationTime/max(numel(corners),1);
row.ErrorMessage='';
end
