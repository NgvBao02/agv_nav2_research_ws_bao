function comparisonResult = compare_path_postprocessors( ...
        plannerName,mapIndex,scenarioIndex)
%COMPARE_PATH_POSTPROCESSORS Mot lenh so sanh dung sau bo hau xu ly.
% Vi du:
%   compare_path_postprocessors('THETA_STAR',1,1)
%   compare_path_postprocessors('NAVFN_ASTAR',4,3)
if nargin<1||isempty(plannerName),plannerName='THETA_STAR';end
if nargin<2||isempty(mapIndex),mapIndex=1;end
if nargin<3||isempty(scenarioIndex),scenarioIndex=1;end
config=defaultCornerOptimizerConfig();
config.enablePlots=false;config.enableAnimation=false;
config.capturePlannerTrace=false;
comparison=defaultPostprocessorComparisonConfig(config);
comparison.plannerName=upper(char(plannerName));
maps=createMapSuite(config);
if mapIndex<1||mapIndex>numel(maps)||mapIndex~=round(mapIndex)
    error('mapIndex phai trong [1,%d].',numel(maps));
end
map=maps(mapIndex);
if scenarioIndex<1||scenarioIndex>numel(map.startGoalPairs)|| ...
        scenarioIndex~=round(scenarioIndex)
    error('scenarioIndex phai trong [1,%d].',numel(map.startGoalPairs));
end
scenario=map.startGoalPairs(scenarioIndex);
timestamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
folderName=sprintf('%s_%s_%s_%s',comparison.plannerName,map.name, ...
    scenario.name,timestamp);
folderName=regexprep(folderName,'[^A-Za-z0-9_-]','_');
outputDirectory=fullfile(comparison.outputRoot,folderName);
if ~exist(outputDirectory,'dir'),mkdir(outputDirectory);end
fprintf(['\nPOSTPROCESSOR BENCHMARK\nPlanner co dinh: %s\n' ...
    'Map/scenario: %s / %s\n'],comparison.plannerName,map.name,scenario.name);
comparisonResult=runPathPostprocessorComparison(comparison.plannerName, ...
    map,scenario,config,comparison);
comparisonResult.outputDirectory=outputDirectory;
disp(comparisonResult.resultTable(:,{'Postprocessor','Implementation', ...
    'PostprocessTime','OutputPathLength','CompletionTime', ...
    'PositionRMSE','MinimumClearance','TaskSuccess'}));
exportPostprocessorComparison(comparisonResult,outputDirectory);
if comparison.saveFigures
    plotPostprocessorComparison(comparisonResult,outputDirectory,comparison);
end
if comparison.enableAnimation
    comparisonResult.animationFile=animatePostprocessorComparison( ...
        comparisonResult,outputDirectory,comparison);
end
save(fullfile(outputDirectory,'postprocessor_comparison.mat'), ...
    'comparisonResult','-v7.3');
fprintf('\nDa luu day du ket qua tai:\n%s\n',outputDirectory);
end
