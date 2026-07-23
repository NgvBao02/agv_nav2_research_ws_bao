function outputs = generateConferencePaperData(outputDirectory)
%GENERATECONFERENCEPAPERDATA Tao du lieu tai lap cho ban thao hoi thao.
% Chay 30 scenario tren 6 map voi:
%   (1) A* truyen thong va A* phat chuyen huong;
%   (2) pivot-only, fixed-radius va adaptive pivot-or-arc;
%   (3) Cung Theta*/NavFn, doi 6 bo hau xu ly de danh gia dung dong gop.
if nargin<1 || isempty(outputDirectory)
    outputDirectory=fullfile(pwd,'results','conference_paper_2026');
end
existingCsv=dir(fullfile(outputDirectory,'*.csv'));
if ~isempty(existingCsv)
    error('PaperData:OutputNotEmpty',[ ...
        'Thu muc da co CSV. Chon outputDirectory moi de khong tron revision: %s'], ...
        outputDirectory);
end
if ~exist(outputDirectory,'dir'),mkdir(outputDirectory);end
figureDirectory=fullfile(outputDirectory,'figures');
if ~exist(figureDirectory,'dir'),mkdir(figureDirectory);end

config=defaultCornerOptimizerConfig();
rng(config.randomSeed,'twister');
runPivotG2Verification();
if ~config.robot.measured
    warning('PaperData:UnmeasuredRobotProfile',[ ...
        'Dang dung %s; chi duoc ghi la ket qua mo phong, khong phai robot that.'], ...
        config.robot.profileName);
end
config.enablePlots=false;
config.enableAnimation=false;
config.enablePlannerAnimation=false;
config.enableRobotAnimation=false;
config.capturePlannerTrace=false;
config.saveFigures=false;
config.numRepeats=1;
config.verbose=true;
config.checkpointAfterEachMap=false;
config.outputDirectory=fullfile(outputDirectory,'ablation');
maps=createMapSuite(config);

fprintf('\n=== 1/3: A* TRADITIONAL VS TURN-PENALTY ===\n');
[astarResults,astarSummary]=runAStarComparison(maps,config);
writetable(astarResults,fullfile(outputDirectory,'astar_comparison_all.csv'));
writetable(astarSummary,fullfile(outputDirectory,'astar_comparison_summary.csv'));

fprintf('\n=== 2/3: MANEUVER ABLATION ===\n');
[ablationResults,~,representatives]=runBatchExperiments(maps,config);
ablationSummary=summarizeAblation(ablationResults);
writetable(ablationResults,fullfile(outputDirectory,'maneuver_ablation_all.csv'));
writetable(ablationSummary,fullfile(outputDirectory,'maneuver_ablation_summary.csv'));

fprintf('\n=== 3/3: FIXED-PLANNER POSTPROCESSOR BENCHMARK ===\n');
comparison=defaultPostprocessorComparisonConfig(config);
comparison.captureSearchTrace=false;
comparison.enableAnimation=false;
comparison.saveAnimation=false;
comparison.saveFigures=false;
postprocessorResults=table();
representativePostprocessor=struct();
caseNumber=0;
plannerNames={'THETA_STAR','NAVFN_ASTAR'};
for plannerIndex=1:numel(plannerNames)
    for mapIndex=1:numel(maps)
        for scenarioIndex=1:numel(maps(mapIndex).startGoalPairs)
            caseNumber=caseNumber+1;
            scenario=maps(mapIndex).startGoalPairs(scenarioIndex);
            fprintf('[%02d/60] %s / %s / %s\n',caseNumber, ...
                plannerNames{plannerIndex},maps(mapIndex).name,scenario.name);
            result=runPathPostprocessorComparison(plannerNames{plannerIndex}, ...
                maps(mapIndex),scenario,config,comparison);
            result.resultTable.RunIndex=repmat(caseNumber,height(result.resultTable),1);
            if isempty(postprocessorResults)
                postprocessorResults=result.resultTable;
            else
                postprocessorResults=[postprocessorResults;result.resultTable]; %#ok<AGROW>
            end
            if plannerIndex==1 && mapIndex==3 && scenarioIndex==1
                representativePostprocessor=result;
            end
        end
    end
end
postprocessorSummary=summarizePostprocessors(postprocessorResults);
writetable(postprocessorResults,fullfile(outputDirectory, ...
    'postprocessor_comparison_all.csv'));
writetable(postprocessorSummary,fullfile(outputDirectory, ...
    'postprocessor_comparison_summary.csv'));

createPaperFigures(maps,representatives,representativePostprocessor,astarSummary, ...
    ablationSummary,postprocessorSummary,figureDirectory,config,comparison);

save(fullfile(outputDirectory,'conference_paper_data.mat'), ...
    'astarResults','astarSummary','ablationResults','ablationSummary', ...
    'postprocessorResults','postprocessorSummary','representatives', ...
    'representativePostprocessor', ...
    'config','comparison','-v7.3');
outputs=struct('outputDirectory',outputDirectory, ...
    'astarResults',astarResults,'astarSummary',astarSummary, ...
    'ablationResults',ablationResults,'ablationSummary',ablationSummary, ...
    'postprocessorResults',postprocessorResults, ...
    'postprocessorSummary',postprocessorSummary, ...
    'figureDirectory',figureDirectory);
fprintf('\nDa tao du lieu bai bao tai:\n%s\n',outputDirectory);
end

function [results,summary]=runAStarComparison(maps,config)
template=struct('MapName','','ScenarioName','','Planner','', ...
    'AlgorithmRevision',config.algorithmRevision, ...
    'RobotProfile',config.robot.profileName, ...
    'Success',false,'PlanningTime',nan,'ExpandedNodes',nan, ...
    'RawWaypoints',nan,'ReducedWaypoints',nan,'NumberOfTurns',nan, ...
    'PathLength',nan,'PathCost',nan,'TurnPenalty',nan,'TurnCostRatio',nan);
rows=repmat(template,2*sum(arrayfun(@(m)numel(m.startGoalPairs),maps)),1);
rowIndex=0;
for mapIndex=1:numel(maps)
    map=maps(mapIndex);
    occupancy=inflateOccupancyGrid(map.occupancy,config.inflationRadius,map.resolution);
    for scenarioIndex=1:numel(map.startGoalPairs)
        scenario=map.startGoalPairs(scenarioIndex);
        modes={'TRADITIONAL_ASTAR','IMPROVED_TURN_PENALTY'};
        for modeIndex=1:numel(modes)
            rowIndex=rowIndex+1;
            localConfig=config;
            localConfig.plannerMode=modes{modeIndex};
            timer=tic;
            [path,info]=planGridPath(occupancy,scenario.start,scenario.goal, ...
                localConfig,false);
            planningTime=toc(timer);
            reduced=removeCollinearPoints(path);
            if size(path,1)>1
                pathLength=sum(hypot(diff(path(:,1)),diff(path(:,2))));
            else
                pathLength=nan;
            end
            if isfield(info,'turnCostRatio'),turnCostRatio=info.turnCostRatio;
            else,turnCostRatio=1+info.turnPenalty;end
            rows(rowIndex)=struct('MapName',map.name,'ScenarioName',scenario.name, ...
                'Planner',modes{modeIndex}, ...
                'AlgorithmRevision',config.algorithmRevision, ...
                'RobotProfile',config.robot.profileName,'Success',info.success, ...
                'PlanningTime',planningTime,'ExpandedNodes',info.expandedNodes, ...
                'RawWaypoints',size(path,1),'ReducedWaypoints',size(reduced,1), ...
                'NumberOfTurns',info.numberOfTurns,'PathLength',pathLength, ...
                'PathCost',info.pathCostCells,'TurnPenalty',info.turnPenalty, ...
                'TurnCostRatio',turnCostRatio);
        end
    end
end
results=struct2table(rows);
summary=summarizeNumeric(results,'Planner', ...
    {'PlanningTime','ExpandedNodes','RawWaypoints','ReducedWaypoints', ...
    'NumberOfTurns','PathLength'});
summary.SuccessRate=groupMean(results,'Planner','Success');
end

function summary=summarizeAblation(results)
fields={'CompletionTime','ActualPathLength','NumberOfFullStops', ...
    'PositionRMSE','HeadingRMSE','MinimumClearance','Jv','Jomega', ...
    'MaximumLeftWheelVelocity','MaximumRightWheelVelocity', ...
    'LimitViolationCount','CollisionCount','ArcSelectionRate', ...
    'TotalAlgorithmTime'};
summary=summarizeNumeric(results,'Method',fields);
summary.SuccessRate=groupMean(results,'Method','Success');
end

function summary=summarizePostprocessors(results)
groups=unique(string(results.Planner)+" | "+string(results.Postprocessor),'stable');
summary=table(groups,'VariableNames',{'PlannerPostprocessor'});
fields={'PostprocessTime','CorePostprocessTime','OutputPathLength', ...
    'IntegratedSquaredCurvature', ...
    'CompletionTime','NumberOfFullStops','PositionRMSE','HeadingRMSE', ...
    'MinimumClearance','Jv','Jomega','MaximumWheelVelocity'};
summary.NumberOfTrials=zeros(numel(groups),1);
summary.TaskSuccessRate=zeros(numel(groups),1);
for f=1:numel(fields)
    summary.(['Mean' fields{f}])=nan(numel(groups),1);
    summary.(['Std' fields{f}])=nan(numel(groups),1);
end
for g=1:numel(groups)
    mask=(string(results.Planner)+" | "+string(results.Postprocessor))==groups(g);
    summary.NumberOfTrials(g)=sum(mask);
    summary.TaskSuccessRate(g)=mean(results.TaskSuccess(mask));
    for f=1:numel(fields)
        field=fields{f};
        summary.(['Mean' field])(g,1)=mean(results.(field)(mask),'omitnan');
        summary.(['Std' field])(g,1)=std(results.(field)(mask),'omitnan');
    end
end
end

function summary=summarizeNumeric(results,groupField,fields)
groups=unique(string(results.(groupField)),'stable');
summary=table(groups,'VariableNames',{groupField});
summary.NumberOfTrials=zeros(numel(groups),1);
for i=1:numel(groups)
    mask=string(results.(groupField))==groups(i);
    summary.NumberOfTrials(i)=sum(mask);
end
for fieldIndex=1:numel(fields)
    field=fields{fieldIndex};
    meanValues=nan(numel(groups),1);
    stdValues=nan(numel(groups),1);
    for i=1:numel(groups)
        mask=string(results.(groupField))==groups(i);
        meanValues(i)=mean(results.(field)(mask),'omitnan');
        stdValues(i)=std(results.(field)(mask),'omitnan');
    end
    summary.(['Mean' field])=meanValues;
    summary.(['Std' field])=stdValues;
end
end

function values=groupMean(results,groupField,valueField)
groups=unique(string(results.(groupField)),'stable');
values=nan(numel(groups),1);
for i=1:numel(groups)
    mask=string(results.(groupField))==groups(i);
    values(i)=mean(double(results.(valueField)(mask)),'omitnan');
end
end

function createPaperFigures(maps,representatives,representativePostprocessor, ...
        astarSummary,ablationSummary,postprocessorSummary,figureDirectory,config,comparison)
oldVisibility=get(groot,'DefaultFigureVisible');
set(groot,'DefaultFigureVisible','off');
cleanup=onCleanup(@()set(groot,'DefaultFigureVisible',oldVisibility));

plotMapSuite(maps);
exportgraphics(gcf,fullfile(figureDirectory,'figure_01_map_suite.png'), ...
    'Resolution',220);close(gcf);

plotBenchmarkSummary(tablevertcat(representatives),representatives,config);
figures=findall(groot,'Type','figure');
for i=1:numel(figures)
    name=get(figures(i),'Name');
    if strcmp(name,'REPRESENTATIVE_PATHS')
        exportgraphics(figures(i),fullfile(figureDirectory, ...
            'figure_02_ablation_paths.png'),'Resolution',220);
    end
end
close(figures);

if ~isempty(fieldnames(representativePostprocessor))
    postFolder=fullfile(figureDirectory,'representative_postprocessor');
    if ~exist(postFolder,'dir'),mkdir(postFolder);end
    comparison.saveFigures=true;
    handles=plotPostprocessorComparison(representativePostprocessor, ...
        postFolder,comparison);
    close(handles);
end

figure('Color','w','Position',[100 100 1250 720]);
tiledlayout(1,3,'TileSpacing','compact','Padding','compact');
nexttile;bar(categorical(astarSummary.Planner),astarSummary.MeanNumberOfTurns);grid on;
ylabel('Mean turns');title('A* heading changes');
nexttile;bar(categorical(ablationSummary.Method),ablationSummary.MeanCompletionTime);grid on;
ylabel('s');title('Maneuver completion time');
nexttile;bar(categorical(ablationSummary.Method),ablationSummary.MeanJomega);grid on;
ylabel('rad/s variation');title('Angular command variation');
exportgraphics(gcf,fullfile(figureDirectory,'figure_03_core_metrics.png'), ...
    'Resolution',220);close(gcf);

figure('Color','w','Position',[100 100 1400 760]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
labels=categorical(postprocessorSummary.PlannerPostprocessor);
nexttile;bar(labels,postprocessorSummary.MeanOutputPathLength);grid on;ylabel('m');title('Output path');
nexttile;bar(labels,postprocessorSummary.MeanCompletionTime);grid on;ylabel('s');title('Completion time');
nexttile;bar(labels,postprocessorSummary.MeanJomega);grid on;ylabel('rad/s variation');title('Jomega');
nexttile;bar(labels,postprocessorSummary.MeanMinimumClearance);grid on;ylabel('m');title('Minimum clearance');
exportgraphics(gcf,fullfile(figureDirectory,'figure_04_postprocessor_summary.png'), ...
    'Resolution',220);close(gcf);
end

function results=tablevertcat(representatives) %#ok<INUSD>
% Dummy table: plotBenchmarkSummary only needs allResults for figures other
% than representative paths. Tao schema nho hop le de ham ve khong loi.
results=table("PIVOT_ONLY",0,0,0,0,0,true,0,0,0,0, ...
    'VariableNames',{'Method','CompletionTime','NumberOfFullStops', ...
    'PositionRMSE','MinimumClearance','Jomega','Success','ArcSelectionRate', ...
    'TimePerMeter','StopsPerCorner','JomegaPerMeter'});
results.MapName="DUMMY";
results.OptimizationTimePerCorner=0;
end
