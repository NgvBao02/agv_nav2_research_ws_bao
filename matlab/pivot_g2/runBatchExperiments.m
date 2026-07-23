function [allResults,methodSummary,representatives] = runBatchExperiments(maps,config)
%RUNBATCHEXPERIMENTS Chay 6x5x3, bat loi va checkpoint sau tung map.
allResults = table();
representatives = cell(numel(maps),1);
if ~exist(config.outputDirectory,'dir'), mkdir(config.outputDirectory); end
for k = 1:numel(maps)
    if config.verbose
        fprintf('\n[%d/%d] %s\n',k,numel(maps),maps(k).name);
    end
    for j = 1:numel(maps(k).startGoalPairs)
        scenario = maps(k).startGoalPairs(j);
        try
            result = runSingleScenario(maps(k),scenario,config);
            rows = result.resultTable;
            if j == config.plotRepresentativeScenarioIndex
                representatives{k} = result;
            end
            if config.verbose
                fprintf('  OK  %-24s\n',scenario.name);
            end
        catch exception
            rows = createFailureResultRows(maps(k),scenario,exception.message,config);
            warning('Batch:ScenarioFailed','%s / %s: %s', ...
                maps(k).name,scenario.name,exception.message);
        end
        if isempty(allResults), allResults=rows; else, allResults=[allResults;rows]; end %#ok<AGROW>
    end
    if config.checkpointAfterEachMap
        checkpointFile = fullfile(config.outputDirectory,'partial_results.mat');
        save(checkpointFile,'allResults','k');
        writetable(allResults,fullfile(config.outputDirectory,'partial_results.csv'));
    end
end
methodSummary = aggregateResults(allResults);
exportResults(allResults,methodSummary,config.outputDirectory);
end
