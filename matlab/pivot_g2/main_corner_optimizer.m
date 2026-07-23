function main_corner_optimizer(mapIndex,scenarioIndex)
%MAIN_CORNER_OPTIMIZER Diem vao cua mo phong pivot-or-arc.
% Goi main_corner_optimizer(4,1) de chay map 4, scenario 1.
% Goi main_corner_optimizer('batch') de chay toan bo benchmark.
config = defaultCornerOptimizerConfig();
config.runMode = 'single';       % 'single' hoac 'batch'
config.singleMapIndex = 1;       % 1..6
config.singleScenarioIndex = 1;  % 1..5
config.enablePlots = true;
config.enableAnimation = true;
config.numRepeats = 10;          % batch: 10; single se tu dong dung 1
config.plannerMode = 'IMPROVED_TURN_PENALTY';
% PAPER: K2=1.94, tuong duong phan phat cong them lambda=0.94.
config.improvedAStar.penaltyMode = 'PAPER';
config.enablePlannerAnimation = true;
config.enableRobotAnimation = true;
config.capturePlannerTrace = config.enableAnimation && config.enablePlannerAnimation;

% Tham so Command Window ghi de cau hinh mac dinh, khong can sua file.
if nargin>=1 && ~isempty(mapIndex)
    if ischar(mapIndex) || (isstring(mapIndex) && isscalar(mapIndex))
        requestedMode=lower(char(mapIndex));
        if ~strcmp(requestedMode,'batch') && ~strcmp(requestedMode,'single')
            error('Tham so chuoi phai la ''single'' hoac ''batch''.');
        end
        config.runMode=requestedMode;
    else
        config.runMode='single';
        config.singleMapIndex=mapIndex;
    end
end
if nargin>=2 && ~isempty(scenarioIndex)
    config.singleScenarioIndex=scenarioIndex;
end

rng(config.randomSeed,'twister');
maps = createMapSuite(config);
switch lower(config.runMode)
    case 'single'
        config.numRepeats = 1;
        validateIndex(config.singleMapIndex,numel(maps),'singleMapIndex');
        selectedMap = maps(config.singleMapIndex);
        validateIndex(config.singleScenarioIndex, ...
            numel(selectedMap.startGoalPairs),'singleScenarioIndex');
        scenario = selectedMap.startGoalPairs(config.singleScenarioIndex);
        result = runSingleScenario(selectedMap,scenario,config);
        disp(result.resultTable);
        if config.enablePlots
            plotMapSuite(maps);
            plotAStarPlannerComparison(result,config);
            plotSimulationResults(result,config);
        end
        if config.enableAnimation && config.enablePlannerAnimation
            animationMap=selectedMap;
            animationMap.occupancy=result.planningOccupancy;
            animateAStarSearch(animationMap,result.astarInfo.trace, ...
                result.rawPath,scenario,config);
        end
        if config.enableAnimation && config.enableRobotAnimation
            animateRobot(selectedMap,result.methodResults(3).simulation,config, ...
                result.methodResults(3).reference);
        end
    case 'batch'
        showPlotsAfterBenchmark = config.enablePlots;
        config.enablePlots = false;
        config.enableAnimation = false;
        config.enablePlannerAnimation = false;
        config.enableRobotAnimation = false;
        config.capturePlannerTrace = false;
        [allResults,methodSummary,representatives] = ...
            runBatchExperiments(maps,config);
        disp(methodSummary);
        if showPlotsAfterBenchmark
            plotMapSuite(maps);
            plotBenchmarkSummary(allResults,representatives,config);
            plotScalabilityResults(allResults);
        end
    otherwise
        error('config.runMode phai la ''single'' hoac ''batch''.');
end
end

function validateIndex(value,maximum,name)
if ~isscalar(value) || value~=round(value) || value<1 || value>maximum
    error('%s phai la so nguyen trong [1,%d].',name,maximum);
end
end
