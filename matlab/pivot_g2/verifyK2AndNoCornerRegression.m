function report = verifyK2AndNoCornerRegression()
%VERIFYK2ANDNOCORNERREGRESSION Kiem tra K2 va 6 duong khong co goc.
config = defaultCornerOptimizerConfig();
config.enablePlots = false;
config.enableAnimation = false;
config.capturePlannerTrace = false;

% PAPER: K2=1.94 la tong chi phi buoc re, lambda phai bang 0.94.
[paperPenalty,paperDetails] = computeAStarTurnPenalty(config);
assert(abs(paperPenalty-0.94)<1e-12, ...
    'PAPER phai tra ve lambda=K2-1=0.94.');
assert(abs(paperDetails.turnCostRatio-1.94)<1e-12, ...
    'Tong chi phi tuong doi cua buoc re PAPER phai la K2=1.94.');

% DYNAMIC: chi cong them ty le thoi gian quay, khong cong trung buoc thang.
dynamicConfig = config;
dynamicConfig.improvedAStar.penaltyMode = 'DYNAMIC';
[dynamicPenalty,dynamicDetails] = computeAStarTurnPenalty(dynamicConfig);
expectedDynamicPenalty = dynamicDetails.turnTime/dynamicDetails.straightTime;
assert(abs(dynamicPenalty-expectedDynamicPenalty)<1e-12, ...
    'DYNAMIC phai tra ve lambda=t_turn/t_straight.');
assert(abs(dynamicDetails.turnCostRatio-(1+dynamicPenalty))<1e-12, ...
    'K2 dong phai bang 1+lambda.');

maps = createMapSuite(config);

% Profile ARC khong duoc de omega ro sang cac mau STRAIGHT.
profileMap = maps(1);
profileScenario = profileMap.startGoalPairs(1);
profileOccupancy = inflateOccupancyGrid(profileMap.occupancy, ...
    config.inflationRadius,profileMap.resolution);
[profilePath,profileInfo] = planGridPath(profileOccupancy, ...
    profileScenario.start,profileScenario.goal,config,false);
assert(profileInfo.success,'Khong tao duoc duong cho kiem tra profile.');
profileReduced = removeCollinearPoints(profilePath);
profileCorners = detectCorners(profileReduced);
[profileDecisions,~] = chooseManeuversForMethod(profileCorners,profileMap, ...
    config,'ADAPTIVE_PIVOT_OR_ARC');
profileReference = generateSpeedProfile(buildReferenceTrajectory( ...
    profileReduced,profileDecisions,config),config);
straightMask = strcmp(profileReference.mode,'STRAIGHT');
assert(all(abs(profileReference.omega(straightMask))<1e-12), ...
    'Profile omega bi ro sang doan STRAIGHT.');
assert(max(abs(profileReference.angularAcceleration)) <= ...
    config.robot.maxAngularAcceleration+1e-9, ...
    'Profile vuot gioi han gia toc goc.');

% Sau day la 6 map/scenario tung loi do decisions rong khong co field valid.
caseIndices = [2 2; 3 2; 3 3; 4 2; 4 3; 5 2];
comparison = defaultNav2ComparisonConfig(config);
comparison.captureSearchTrace = false;
comparison.enableAnimation = false;
comparison.saveAnimation = false;
comparison.saveFigures = false;

mapNames = strings(size(caseIndices,1),1);
scenarioNames = strings(size(caseIndices,1),1);
cornerCounts = zeros(size(caseIndices,1),1);
planningSuccess = false(size(caseIndices,1),1);
for i = 1:size(caseIndices,1)
    map = maps(caseIndices(i,1));
    scenario = map.startGoalPairs(caseIndices(i,2));
    result = runNav2PlannerComparison(map,scenario,config,comparison);
    proposedIndex = find(arrayfun(@(r)isfield(r.planner,'name') && ...
        strcmp(r.planner.name,'PROPOSED_ADAPTIVE'),result.runs),1);
    assert(~isempty(proposedIndex),'Khong tim thay dong PROPOSED_ADAPTIVE.');
    proposedRun = result.runs(proposedIndex);
    assert(isempty(proposedRun.error), ...
        'PROPOSED_ADAPTIVE bi loi tren %s/%s: %s', ...
        map.name,scenario.name,proposedRun.error);
    assert(isempty(proposedRun.decisions), ...
        'Ca hoi quy nay du kien la duong khong co goc.');
    mapNames(i) = string(map.name);
    scenarioNames(i) = string(scenario.name);
    cornerCounts(i) = numel(proposedRun.decisions);
    planningSuccess(i) = proposedRun.planner.success;
end
assert(all(planningSuccess),'Mot hoac nhieu ca khong co goc van that bai.');

report = table(mapNames,scenarioNames,cornerCounts,planningSuccess, ...
    'VariableNames',{'Map','Scenario','CornerCount','Success'});
fprintf('PAPER: K2=%.4f, lambda=%.4f\n', ...
    paperDetails.turnCostRatio,paperPenalty);
fprintf('DYNAMIC: K2=%.4f, lambda=%.4f\n', ...
    dynamicDetails.turnCostRatio,dynamicPenalty);
disp(report);
end
