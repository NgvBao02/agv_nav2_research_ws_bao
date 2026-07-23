function exportNav2Comparison(comparisonResult,outputDirectory)
%EXPORTNAV2COMPARISON Luu CSV, MAT va ban tom tat tu dong.
if ~exist(outputDirectory,'dir'),mkdir(outputDirectory);end
T=comparisonResult.resultTable;
writetable(T,fullfile(outputDirectory,'planner_comparison.csv'));
save(fullfile(outputDirectory,'planner_comparison.mat'),'comparisonResult','T','-v7.3');
fileId=fopen(fullfile(outputDirectory,'comparison_summary.txt'),'w','n','UTF-8');
if fileId<0,warning('Khong tao duoc comparison_summary.txt');return;end
cleanup=onCleanup(@()fclose(fileId));
fprintf(fileId,'NAV2-STYLE GLOBAL PLANNER COMPARISON\n');
fprintf(fileId,'Map: %s\nScenario: %s\n\n',comparisonResult.map.name,comparisonResult.scenario.name);
fprintf(fileId,['IMPORTANT: ROS 2/WSL is not installed on this computer. NavFn, Smac and ' ...
    'Theta* rows are MATLAB-equivalent research implementations, not runtime measurements ' ...
    'of the C++ Nav2 plugins.\n\n']);
fprintf(fileId,['COMMON POST-PROCESSING: every planner uses the same adaptive ' ...
    'corner fillet / collision-checked numerical smoother and the same ' ...
    'projection-based trajectory tracker.\n\n']);
valid=T.TaskSuccess;
if any(valid)
    [~,bestPlan]=min(T.PlanningTime(valid));indices=find(valid);bestPlan=indices(bestPlan);
    [~,bestMotion]=min(T.CompletionTime(valid));bestMotion=indices(bestMotion);
    [~,bestClearance]=max(T.MinimumClearance(valid));bestClearance=indices(bestClearance);
    [~,bestRmse]=min(T.PositionRMSE(valid));bestRmse=indices(bestRmse);
    fprintf(fileId,'Fastest planning: %s (%.6f s)\n',T.Planner{bestPlan},T.PlanningTime(bestPlan));
    fprintf(fileId,'Fastest completion: %s (%.3f s)\n',T.Planner{bestMotion},T.CompletionTime(bestMotion));
    fprintf(fileId,'Largest clearance: %s (%.3f m)\n',T.Planner{bestClearance},T.MinimumClearance(bestClearance));
    fprintf(fileId,'Smallest position RMSE: %s (%.4f m)\n\n',T.Planner{bestRmse},T.PositionRMSE(bestRmse));
end
fprintf(fileId,'PLANNER-SPECIFIC INTERPRETATION\n');
for i=1:height(T)
    fprintf(fileId,'- %s: success=%d, plan=%.6f s, completion=%.3f s, clearance=%.3f m. %s\n', ...
        T.Planner{i},T.TaskSuccess(i),T.PlanningTime(i),T.CompletionTime(i), ...
        T.MinimumClearance(i),T.Notes{i});
end
end
