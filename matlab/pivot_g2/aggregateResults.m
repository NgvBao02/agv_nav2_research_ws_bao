function methodSummary = aggregateResults(allResults)
%AGGREGATERESULTS Tong hop cac chi so theo tung phuong phap.
if isempty(allResults)
    methodSummary = table();
    return;
end
methods = unique(string(allResults.Method),'stable');
rows = repmat(struct('Method','', 'NumberOfTrials',0, 'SuccessRate',nan, ...
    'MeanCompletionTime',nan,'MeanActualPathLength',nan, ...
    'MeanNumberOfStops',nan,'MeanPositionRMSE',nan, ...
    'MeanMinimumClearance',nan,'MeanJv',nan,'MeanJomega',nan, ...
    'MeanComputationTime',nan),numel(methods),1);
for i = 1:numel(methods)
    mask = string(allResults.Method)==methods(i);
    valid = mask & ~isnan(allResults.CompletionTime);
    rows(i).Method = char(methods(i));
    rows(i).NumberOfTrials = sum(mask);
    rows(i).SuccessRate = mean(allResults.Success(mask));
    rows(i).MeanCompletionTime = mean(allResults.CompletionTime(valid),'omitnan');
    rows(i).MeanActualPathLength = mean(allResults.ActualPathLength(valid),'omitnan');
    rows(i).MeanNumberOfStops = mean(allResults.NumberOfFullStops(valid),'omitnan');
    rows(i).MeanPositionRMSE = mean(allResults.PositionRMSE(valid),'omitnan');
    rows(i).MeanMinimumClearance = mean(allResults.MinimumClearance(valid),'omitnan');
    rows(i).MeanJv = mean(allResults.Jv(valid),'omitnan');
    rows(i).MeanJomega = mean(allResults.Jomega(valid),'omitnan');
    rows(i).MeanComputationTime = mean(allResults.TotalAlgorithmTime(valid),'omitnan');
end
methodSummary = struct2table(rows);
end
