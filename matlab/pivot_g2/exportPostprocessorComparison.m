function exportPostprocessorComparison(result,outputDirectory)
%EXPORTPOSTPROCESSORCOMPARISON CSV/MAT/manifest/decision va tom tat.
if ~exist(outputDirectory,'dir'),mkdir(outputDirectory);end
T=result.resultTable;
writetable(T,fullfile(outputDirectory,'postprocessor_comparison.csv'));
writetable(decisionTable(result.runs), ...
    fullfile(outputDirectory,'corner_decisions.csv'));
save(fullfile(outputDirectory,'postprocessor_comparison_data.mat'), ...
    'result','T','-v7.3');
writeManifest(result,fullfile(outputDirectory,'experiment_manifest.txt'));
writeSummary(result,fullfile(outputDirectory,'comparison_summary.txt'));
end

function T=decisionTable(runs)
rows=struct([]);
for i=1:numel(runs)
    decisions=runs(i).decisions;
    for j=1:numel(decisions)
        d=decisions(j);
        [curveType,curvatureEnergy]=selectedCurveData(d);
        row=struct('Postprocessor',runs(i).method.name,'CornerIndex',j, ...
            'TurnAngleDeg',d.corner.turnAngle*180/pi, ...
            'TurnDirection',d.corner.turnDirection, ...
            'SelectedType',d.selectedType,'SelectedRadius',d.selectedRadius, ...
            'SelectedTime',d.selectedTime, ...
            'SelectedClearance',d.selectedClearance, ...
            'PivotTime',d.pivotTime,'FastestArcTime',d.bestArcTime, ...
            'CompetitiveArcCandidates',readField(d,'competitiveArcCandidates',0), ...
            'SelectionScore',readField(d,'selectionScore',nan), ...
            'CurveType',curveType,'CandidateCurvatureEnergy',curvatureEnergy, ...
            'Reason',d.reason);
        if isempty(rows),rows=row;else,rows(end+1,1)=row;end %#ok<AGROW>
    end
end
if isempty(rows)
    T=table('Size',[0 15], ...
        'VariableTypes',{'string','double','double','string','string', ...
        'double','double','double','double','double','double','double', ...
        'string','double','string'}, ...
        'VariableNames',{'Postprocessor','CornerIndex','TurnAngleDeg', ...
        'TurnDirection','SelectedType','SelectedRadius','SelectedTime', ...
        'SelectedClearance','PivotTime','FastestArcTime', ...
        'CompetitiveArcCandidates','SelectionScore','CurveType', ...
        'CandidateCurvatureEnergy','Reason'});
else,T=struct2table(rows);end
end

function writeManifest(result,fileName)
fid=fopen(fileName,'w','n','UTF-8');
if fid<0,warning('Khong tao duoc %s.',fileName);return;end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
c=result.comparisonConfig;
fprintf(fid,'POSTPROCESSOR COMPARISON EXPERIMENT MANIFEST\n');
fprintf(fid,'Generated: %s\n',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
fprintf(fid,'Planner: %s\nPlanner plugin: %s\n',result.planner.name,result.planner.plugin);
fprintf(fid,'Planner implementation: %s\n',result.planner.implementation);
fprintf(fid,'Map: %s\nScenario: %s\n',result.map.name,result.scenario.name);
fprintf(fid,'Algorithm revision: %s\n',result.algorithmRevision);
fprintf(fid,'Robot profile: %s\nRobot profile measured: %d\n', ...
    result.robotConfig.profileName,result.robotConfig.measured);
fprintf(fid,'Input path signature: %s\n',result.inputPathSignature);
fprintf(fid,'Fairness passed: %d\nSame planner run: %d\nSame input path: %d\n', ...
    result.fairness.passed,result.fairness.samePlannerRun, ...
    result.fairness.sameInputPathForAll);
fprintf(fid,'Same controller: %d\nController signature: %s\n\n', ...
    result.fairness.sameControllerForAll,result.fairness.controllerSignature);
fprintf(fid,'METHODS\n');
for i=1:numel(result.runs)
    fprintf(fid,'- %s | %s | %s\n',result.runs(i).method.name, ...
        result.runs(i).method.plugin,result.runs(i).method.implementation);
    if ~isempty(fieldnames(result.runs(i).postprocessInfo))
        fprintf(fid,'  preprocessing: %s\n', ...
            result.runs(i).postprocessInfo.preprocessing);
    end
end
fprintf(fid,'\nNAV2 SIMPLE PARAMETERS\n');writeStruct(fid,c.simple);
fprintf(fid,'\nNAV2 SAVITZKY-GOLAY PARAMETERS\n');writeStruct(fid,c.savitzkyGolay);
fprintf(fid,'\nCONSTRAINED MATLAB-EQUIVALENT PARAMETERS\n');writeStruct(fid,c.constrained);
fprintf(fid,'\nPROPOSED SELECTION PARAMETERS\n');writeStruct(fid,c.proposed);
fprintf(fid,'\nCOMMON-WINDOW TIME PARAMETERS\n');
writeStruct(fid,result.timeComparisonConfig);
fprintf(fid,'\nPROPOSED REFINEMENT PARAMETERS\n');
writeStruct(fid,c.proposedRefinement);
fprintf(fid,['\nSCIENTIFIC SCOPE\nSimple and Savitzky-Golay reproduce the ' ...
    'documented Nav2 update/filter equations. Constrained is a deterministic ' ...
    'MATLAB objective-equivalent and is not the Ceres C++ runtime plugin. ' ...
    'All timings are MATLAB measurements on this computer.\n']);
end

function writeSummary(result,fileName)
fid=fopen(fileName,'w','n','UTF-8');
if fid<0,warning('Khong tao duoc %s.',fileName);return;end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
T=result.resultTable;
fprintf(fid,'FIXED-PLANNER POSTPROCESSOR COMPARISON\n');
fprintf(fid,'Planner: %s\nMap/scenario: %s / %s\n\n', ...
    result.planner.name,result.map.name,result.scenario.name);
fprintf(fid,['FAIRNESS: planner chay dung mot lan; moi method nhan path co ' ...
    'signature %s; controller va gioi han robot giong nhau.\n\n'], ...
    result.inputPathSignature);
valid=T.PostprocessSuccess;
successful=valid&T.TaskSuccess;
if any(successful)
    indices=find(successful);
    [~,q]=min(T.CompletionTime(successful));bestTime=indices(q);
    [~,q]=min(T.IntegratedSquaredCurvature(successful));bestSmooth=indices(q);
    [~,q]=min(T.PositionRMSE(successful));bestTracking=indices(q);
    [~,q]=max(T.MinimumClearance(successful));bestClear=indices(q);
    fprintf(fid,'Fastest completion: %s (%.3f s)\n', ...
        T.Postprocessor{bestTime},T.CompletionTime(bestTime));
    fprintf(fid,'Lowest curvature energy: %s (%.6f 1/m)\n', ...
        T.Postprocessor{bestSmooth},T.IntegratedSquaredCurvature(bestSmooth));
    fprintf(fid,'Lowest tracking RMSE: %s (%.4f m)\n', ...
        T.Postprocessor{bestTracking},T.PositionRMSE(bestTracking));
    fprintf(fid,'Largest actual clearance: %s (%.4f m)\n\n', ...
        T.Postprocessor{bestClear},T.MinimumClearance(bestClear));
end
fprintf(fid,'PER-METHOD RESULTS\n');
for i=1:height(T)
    fprintf(fid,['- %s [%s]: post=%.6f s, completion=%.3f s, ' ...
        'I(k^2)=%.6f, stops=%.0f, RMSE=%.4f m, clearance=%.4f m, ' ...
        'success=%d. %s\n'],T.Postprocessor{i},T.Implementation{i}, ...
        T.PostprocessTime(i),T.CompletionTime(i), ...
        T.IntegratedSquaredCurvature(i),T.NumberOfFullStops(i), ...
        T.PositionRMSE(i),T.MinimumClearance(i),T.TaskSuccess(i),T.Notes{i});
end
fprintf(fid,['\nINTERPRETATION RULE: khong ket luan tu mot map. Dung batch summary ' ...
    'va paired deltas tren tat ca scenario de viet bai.\n']);
end

function writeStruct(fid,s)
fields=fieldnames(s);
for i=1:numel(fields)
    value=s.(fields{i});
    if isnumeric(value)&&isscalar(value),text=sprintf('%.12g',value);
    elseif islogical(value)&&isscalar(value),text=sprintf('%d',value);
    elseif ischar(value),text=value;
    else,text=mat2str(value);end
    fprintf(fid,'%s: %s\n',fields{i},text);
end
end
function value=readField(s,name,defaultValue)
if isfield(s,name),value=s.(name);else,value=defaultValue;end
end
function [curveType,energy]=selectedCurveData(decision)
curveType='PIVOT';energy=nan;
if ~strcmp(decision.selectedType,'ARC')||isnan(decision.bestArcIndex),return;end
candidate=decision.arcCandidates(decision.bestArcIndex);
curveType=readField(candidate,'curveType','CIRCULAR_ARC');
energy=readField(candidate,'curvatureEnergy', ...
    abs(decision.corner.turnAngle)/max(candidate.radius,eps));
end
