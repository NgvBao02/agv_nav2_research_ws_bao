function exportResults(allResults, methodSummary, outputDirectory)
%EXPORTRESULTS Ghi CSV va MAT; tao thu muc neu chua ton tai.
if ~exist(outputDirectory,'dir')
    mkdir(outputDirectory);
end
writetable(allResults,fullfile(outputDirectory,'all_results.csv'));
writetable(methodSummary,fullfile(outputDirectory,'method_summary.csv'));
save(fullfile(outputDirectory,'all_results.mat'),'allResults','methodSummary');
end
