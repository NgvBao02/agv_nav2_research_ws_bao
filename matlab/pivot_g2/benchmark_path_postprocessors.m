function benchmark = benchmark_path_postprocessors( ...
        plannerNames,mapIndices,scenarioIndices)
%BENCHMARK_PATH_POSTPROCESSORS Batch Theta*/NavFn tren nhieu map.
% Mac dinh: THETA_STAR va NAVFN_ASTAR, tat ca 6 map x 5 scenario.
if nargin<1||isempty(plannerNames),plannerNames={'THETA_STAR','NAVFN_ASTAR'};end
if ischar(plannerNames)||isstring(plannerNames),plannerNames=cellstr(plannerNames);end
config=defaultCornerOptimizerConfig();config.enablePlots=false;
config.enableAnimation=false;config.capturePlannerTrace=false;
comparison=defaultPostprocessorComparisonConfig(config);
comparison.enableAnimation=false;comparison.saveAnimation=false;
comparison.saveFigures=false;
maps=createMapSuite(config);
if nargin<2||isempty(mapIndices),mapIndices=1:numel(maps);end
if nargin<3||isempty(scenarioIndices),scenarioIndices=[];end
timestamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
outputDirectory=fullfile(comparison.batchOutputRoot,timestamp);
if ~exist(outputDirectory,'dir'),mkdir(outputDirectory);end
allTables=cell(0,1);caseResults=cell(0,1);caseIndex=0;
for p=1:numel(plannerNames)
    plannerName=upper(char(plannerNames{p}));
    for m=mapIndices
        if m<1||m>numel(maps),error('mapIndex %d khong hop le.',m);end
        map=maps(m);
        if isempty(scenarioIndices),cases=1:numel(map.startGoalPairs);
        else,cases=scenarioIndices;end
        for s=cases
            if s<1||s>numel(map.startGoalPairs)
                error('scenarioIndex %d khong hop le cho map %s.',s,map.name);
            end
            caseIndex=caseIndex+1;scenario=map.startGoalPairs(s);
            fprintf('[%d] %s | %s | %s\n',caseIndex,plannerName,map.name,scenario.name);
            try
                one=runPathPostprocessorComparison(plannerName,map,scenario, ...
                    config,comparison);
                one.resultTable.RunIndex=repmat(caseIndex,height(one.resultTable),1);
                allTables{end+1,1}=one.resultTable; %#ok<AGROW>
                caseResults{end+1,1}=one; %#ok<AGROW>
                caseFolder=fullfile(outputDirectory,sprintf('%03d_%s_%s_%s', ...
                    caseIndex,plannerName,map.name,scenario.name));
                mkdir(caseFolder);exportPostprocessorComparison(one,caseFolder);
            catch exception
                warning('Benchmark:CaseFailed','%s/%s/%s: %s', ...
                    plannerName,map.name,scenario.name,exception.message);
            end
        end
    end
end
if isempty(allTables),error('Khong co ca benchmark nao thanh cong.');end
allResults=vertcat(allTables{:});
summary=aggregatePostprocessorBenchmark(allResults);
pairedSummary=pairedPostprocessorSummary(allResults);
writetable(allResults,fullfile(outputDirectory,'all_postprocessor_runs.csv'));
writetable(summary,fullfile(outputDirectory,'aggregate_postprocessor_summary.csv'));
writetable(pairedSummary,fullfile(outputDirectory,'paired_vs_proposed.csv'));
save(fullfile(outputDirectory,'postprocessor_benchmark.mat'), ...
    'allResults','summary','pairedSummary','caseResults','comparison','config','-v7.3');
benchmark=struct('allResults',allResults,'summary',summary, ...
    'pairedSummary',pairedSummary,'caseResults',{caseResults}, ...
    'outputDirectory',outputDirectory);
fprintf('\nDa luu batch benchmark tai:\n%s\n',outputDirectory);
disp(summary);
end

function summary=aggregatePostprocessorBenchmark(T)
planners=unique(T.Planner,'stable');methods=unique(T.Postprocessor,'stable');
rows=struct([]);
for p=1:numel(planners)
    for m=1:numel(methods)
        mask=strcmp(T.Planner,planners{p})&strcmp(T.Postprocessor,methods{m});
        if ~any(mask),continue;end
        row=struct('Planner',planners{p},'Postprocessor',methods{m}, ...
            'Runs',sum(mask),'TaskSuccessRate',mean(T.TaskSuccess(mask)), ...
            'FallbackRate',mean(T.FallbackUsed(mask)), ...
            'MeanPostprocessTime',mean(T.PostprocessTime(mask),'omitnan'), ...
            'MeanCorePostprocessTime',mean(T.CorePostprocessTime(mask),'omitnan'), ...
            'MeanCompletionTime',mean(T.CompletionTime(mask),'omitnan'), ...
            'MeanCurvatureEnergy',mean(T.IntegratedSquaredCurvature(mask),'omitnan'), ...
            'MeanFullStops',mean(T.NumberOfFullStops(mask),'omitnan'), ...
            'MeanPositionRMSE',mean(T.PositionRMSE(mask),'omitnan'), ...
            'MeanMinimumClearance',mean(T.MinimumClearance(mask),'omitnan'), ...
            'MeanJomega',mean(T.Jomega(mask),'omitnan'));
        if isempty(rows),rows=row;else,rows(end+1,1)=row;end %#ok<AGROW>
    end
end
summary=struct2table(rows);
end

function summary=pairedPostprocessorSummary(T)
planners=unique(T.Planner,'stable');baselines=unique(T.Postprocessor,'stable');
baselines(strcmp(baselines,'PROPOSED_PIVOT_ARC'))=[];rows=struct([]);
for p=1:numel(planners)
    plannerMask=strcmp(T.Planner,planners{p});
    proposed=T(plannerMask&strcmp(T.Postprocessor,'PROPOSED_PIVOT_ARC'),:);
    for b=1:numel(baselines)
        baseline=T(plannerMask&strcmp(T.Postprocessor,baselines{b}),:);
        [common,ia,ib]=intersect(proposed.RunIndex,baseline.RunIndex,'stable');
        if isempty(common),continue;end
        P=proposed(ia,:);B=baseline(ib,:);valid=P.TaskSuccess&B.TaskSuccess;
        row=struct('Planner',planners{p},'Baseline',baselines{b}, ...
            'PairedRuns',numel(common),'BothSuccessfulRuns',sum(valid), ...
            'ProposedSuccessRate',mean(P.TaskSuccess), ...
            'BaselineSuccessRate',mean(B.TaskSuccess), ...
            'MeanDeltaCompletionTime',pairedMean(P.CompletionTime,B.CompletionTime,valid), ...
            'MeanDeltaCurvatureEnergy',pairedMean(P.IntegratedSquaredCurvature, ...
                B.IntegratedSquaredCurvature,valid), ...
            'MeanDeltaFullStops',pairedMean(P.NumberOfFullStops,B.NumberOfFullStops,valid), ...
            'MeanDeltaPositionRMSE',pairedMean(P.PositionRMSE,B.PositionRMSE,valid), ...
            'MeanDeltaMinimumClearance',pairedMean(P.MinimumClearance,B.MinimumClearance,valid), ...
            'MeanDeltaJomega',pairedMean(P.Jomega,B.Jomega,valid), ...
            'CompletionTimeWinRate',winRate(P.CompletionTime,B.CompletionTime,valid,-1), ...
            'CurvatureEnergyWinRate',winRate(P.IntegratedSquaredCurvature, ...
                B.IntegratedSquaredCurvature,valid,-1), ...
            'ClearanceWinRate',winRate(P.MinimumClearance,B.MinimumClearance,valid,1));
        if isempty(rows),rows=row;else,rows(end+1,1)=row;end %#ok<AGROW>
    end
end
summary=struct2table(rows);
end
function value=pairedMean(proposed,baseline,valid)
delta=proposed(valid)-baseline(valid);value=mean(delta,'omitnan');
end
function value=winRate(proposed,baseline,valid,direction)
delta=direction*(proposed(valid)-baseline(valid));
delta=delta(isfinite(delta));
if isempty(delta),value=nan;else,value=mean(delta>0);end
end
