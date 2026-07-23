function [decisionTable,radiusSummary] = collectConferenceCornerDecisions(outputDirectory)
%COLLECTCONFERENCECORNERDECISIONS Tong hop lua chon pivot/arc tren 30 ca.
if nargin<1 || isempty(outputDirectory)
    outputDirectory=fullfile(pwd,'results','conference_paper_2026');
end
if ~exist(outputDirectory,'dir'),mkdir(outputDirectory);end
config=defaultCornerOptimizerConfig();
config.capturePlannerTrace=false;
maps=createMapSuite(config);
rows=struct([]);
for mapIndex=1:numel(maps)
    map=maps(mapIndex);
    occupancy=inflateOccupancyGrid(map.occupancy,config.inflationRadius,map.resolution);
    for scenarioIndex=1:numel(map.startGoalPairs)
        scenario=map.startGoalPairs(scenarioIndex);
        [path,info]=planGridPath(occupancy,scenario.start,scenario.goal,config,false);
        if ~info.success,error('%s/%s: %s',map.name,scenario.name,info.message);end
        reduced=removeCollinearPoints(path);
        corners=detectCorners(reduced);
        [decisions,~]=chooseManeuversForMethod(corners,map,config, ...
            'ADAPTIVE_PIVOT_OR_ARC');
        for cornerIndex=1:numel(decisions)
            d=decisions(cornerIndex);
            row=struct('AlgorithmRevision',config.algorithmRevision, ...
                'RobotProfile',config.robot.profileName, ...
                'MapName',map.name,'ScenarioName',scenario.name, ...
                'CornerIndex',cornerIndex,'TurnAngleRad',d.corner.turnAngle, ...
                'SelectedType',d.selectedType,'SelectedRadius',d.selectedRadius, ...
                'SelectedPredictedTime',d.selectedTime, ...
                'SelectedClearance',d.selectedClearance, ...
                'PivotPredictedTime',d.pivotTime,'BestArcPredictedTime',d.bestArcTime, ...
                'RejectedArcCandidates',d.rejectedArcCandidates,'Reason',d.reason);
            if isempty(rows),rows=row;else,rows(end+1,1)=row;end %#ok<AGROW>
        end
    end
end
if isempty(rows)
    decisionTable=table();radiusSummary=table();return;
end
decisionTable=struct2table(rows);
keys=unique(string(decisionTable.SelectedType)+"_R"+ ...
    compose('%.2f',decisionTable.SelectedRadius),'stable');
counts=zeros(numel(keys),1);
for i=1:numel(keys)
    labels=string(decisionTable.SelectedType)+"_R"+ ...
        compose('%.2f',decisionTable.SelectedRadius);
    counts(i)=sum(labels==keys(i));
end
radiusSummary=table(keys,counts,counts/height(decisionTable), ...
    'VariableNames',{'Decision','Count','Fraction'});
writetable(decisionTable,fullfile(outputDirectory,'corner_decisions_all.csv'));
writetable(radiusSummary,fullfile(outputDirectory,'corner_decision_summary.csv'));
save(fullfile(outputDirectory,'corner_decisions.mat'), ...
    'decisionTable','radiusSummary');
end
