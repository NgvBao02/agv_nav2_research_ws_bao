function [decisions, timing] = chooseManeuversForMethod(corners,map,config,method)
%CHOOSEMANEUVERSFORMETHOD Danh gia tat ca cac goc bang mot phuong phap.
decisions = struct([]);
totalFootprintTime = 0;
tAll = tic;
for i = 1:numel(corners)
    decision = selectCornerManeuver(corners(i),map,config,method);
    if i == 1
        decisions = decision;
    else
        decisions(i,1) = decision; %#ok<AGROW>
    end
    totalFootprintTime = totalFootprintTime + ...
        decision.pivot.footprintCheckTime;
    if ~isempty(decision.arcCandidates)
        totalFootprintTime = totalFootprintTime + ...
            sum([decision.arcCandidates.footprintCheckTime]);
    end
end
totalTime = toc(tAll);
if isempty(decisions)
    candidateCount = 0;
    rejectedCount = 0;
else
    candidateCount = sum(arrayfun(@(d)numel(d.arcCandidates),decisions));
    rejectedCount = sum(arrayfun(@(d)d.rejectedArcCandidates,decisions));
end
timing = struct('cornerOptimizationTime',totalTime, ...
    'footprintCheckTime',totalFootprintTime, ...
    'maneuverSelectionTime',max(0,totalTime-totalFootprintTime), ...
    'numberOfArcCandidates',candidateCount, ...
    'numberOfRejectedArcCandidates',rejectedCount);
end
