function tuning = tune_proposed_postprocessor(bezierFractions,timeSlacks, ...
        trainingMapIndices,validationMapIndices,scenarioIndices)
%TUNE_PROPOSED_POSTPROCESSOR Sweep train/validation, khong sua default.
% Vi du nhanh:
%   tune_proposed_postprocessor([.30 .35 .40],[.15 .20 .25])
if nargin<1||isempty(bezierFractions),bezierFractions=[0.25 0.30 0.35 0.40];end
if nargin<2||isempty(timeSlacks),timeSlacks=[0.10 0.15 0.20 0.25 0.30];end
if nargin<3||isempty(trainingMapIndices),trainingMapIndices=[1 3 5];end
if nargin<4||isempty(validationMapIndices),validationMapIndices=[2 4 6];end
if nargin<5||isempty(scenarioIndices),scenarioIndices=1;end
config=defaultCornerOptimizerConfig();config.enablePlots=false;
config.enableAnimation=false;config.capturePlannerTrace=false;
comparison=defaultPostprocessorComparisonConfig(config);
comparison.enableAnimation=false;comparison.saveFigures=false;
maps=createMapSuite(config);allMapIndices=[trainingMapIndices validationMapIndices];

% Planner chay mot lan cho moi case va duoc tai su dung cho moi tham so.
cases=cell(0,1);caseNumber=0;
for mapIndex=allMapIndices
    for scenarioIndex=scenarioIndices
        caseNumber=caseNumber+1;scenario=maps(mapIndex).startGoalPairs(scenarioIndex);
        [planner,occupancy]=planPostprocessorInputPath('THETA_STAR', ...
            maps(mapIndex),scenario,config,comparison);
        cases{caseNumber,1}=struct('mapIndex',mapIndex,'map',maps(mapIndex), ...
            'scenario',scenario,'planner',planner,'occupancy',occupancy, ...
            'split',ternary(ismember(mapIndex,trainingMapIndices),'TRAIN','VALIDATION')); %#ok<AGROW>
    end
end

rows=struct([]);runIndex=0;
for f=1:numel(bezierFractions)
    for s=1:numel(timeSlacks)
        localConfig=config;
        localConfig.adaptiveSelection.bezierControlFraction=bezierFractions(f);
        localConfig.adaptiveSelection.timeCompetitiveSlack=timeSlacks(s);
        localComparison=comparison;
        localComparison.proposed=localConfig.adaptiveSelection;
        for q=1:numel(cases)
            runIndex=runIndex+1;one=cases{q};inputPath=one.planner.path;
            try
                [reference,~,decisions,info]=applyPathPostprocessor( ...
                    'PROPOSED_PIVOT_ARC',inputPath,one.occupancy,one.map, ...
                    localConfig,localComparison);
                simulation=simulateDifferentialDrive(reference,one.map,localConfig);
                metrics=computeTrackingMetrics(simulation,reference,decisions,localConfig);
                curvatureEnergy=referenceCurvatureEnergy(reference);
                success=metrics.Success;message='OK';
            catch exception
                metrics=emptyMetrics();curvatureEnergy=nan;success=false;
                info=struct('postprocessTime',nan,'coreTime',nan);message=exception.message;
            end
            row=struct('BezierFraction',bezierFractions(f), ...
                'TimeSlack',timeSlacks(s),'Split',one.split, ...
                'MapName',one.map.name,'ScenarioName',one.scenario.name, ...
                'Success',success,'CompletionTime',metrics.CompletionTime, ...
                'CurvatureEnergy',curvatureEnergy,'PositionRMSE',metrics.PositionRMSE, ...
                'MinimumClearance',metrics.MinimumClearance,'Jomega',metrics.Jomega, ...
                'FullStops',metrics.NumberOfFullStops, ...
                'PostprocessTime',info.postprocessTime,'CoreTime',info.coreTime, ...
                'Message',message);
            if isempty(rows),rows=row;else,rows(end+1,1)=row;end %#ok<AGROW>
        end
    end
end
runs=struct2table(rows);summary=summarizeCandidates(runs,config);
timestamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
outputDirectory=fullfile(config.outputDirectory,'postprocessor_tuning',timestamp);
mkdir(outputDirectory);
writetable(runs,fullfile(outputDirectory,'tuning_runs.csv'));
writetable(summary,fullfile(outputDirectory,'candidate_summary.csv'));
save(fullfile(outputDirectory,'tuning_data.mat'),'runs','summary','config', ...
    'comparison','-v7.3');
best=summary(summary.SelectedByTraining,:);
tuning=struct('runs',runs,'summary',summary,'selected',best, ...
    'outputDirectory',outputDirectory);
fprintf('\nTUNING COMPLETE: %s\n',outputDirectory);disp(best);
end

function summary=summarizeCandidates(runs,config)
pairs=unique(runs(:,{'BezierFraction','TimeSlack'}),'rows','stable');
rows=struct([]);
for i=1:height(pairs)
    mask=runs.BezierFraction==pairs.BezierFraction(i)& ...
        runs.TimeSlack==pairs.TimeSlack(i);
    for split={'TRAIN','VALIDATION'}
        local=mask&strcmp(runs.Split,split{1});
        row=struct('BezierFraction',pairs.BezierFraction(i), ...
            'TimeSlack',pairs.TimeSlack(i),'Split',split{1}, ...
            'Runs',sum(local),'SuccessRate',mean(runs.Success(local)), ...
            'MeanCompletionTime',mean(runs.CompletionTime(local),'omitnan'), ...
            'MeanCurvatureEnergy',mean(runs.CurvatureEnergy(local),'omitnan'), ...
            'MeanPositionRMSE',mean(runs.PositionRMSE(local),'omitnan'), ...
            'MinimumObservedClearance',min(runs.MinimumClearance(local),[],'omitnan'), ...
            'MeanJomega',mean(runs.Jomega(local),'omitnan'), ...
            'MeanFullStops',mean(runs.FullStops(local),'omitnan'), ...
            'MeanCoreTime',mean(runs.CoreTime(local),'omitnan'), ...
            'Feasible',false,'Pareto',false,'TrainingScore',nan, ...
            'SelectedByTraining',false);
        row.Feasible=row.SuccessRate==1&& ...
            row.MinimumObservedClearance>=config.robot.clearanceSafe-1e-9&& ...
            row.MeanFullStops<=0.1;
        if isempty(rows),rows=row;else,rows(end+1,1)=row;end %#ok<AGROW>
    end
end
summary=struct2table(rows);train=strcmp(summary.Split,'TRAIN')&summary.Feasible;
indices=find(train);
if isempty(indices),warning('Tuning:NoFeasible','Khong co cau hinh train kha thi.');return;end
objective=[summary.MeanCurvatureEnergy(indices), ...
    summary.MeanCompletionTime(indices),summary.MeanPositionRMSE(indices), ...
    summary.MeanJomega(indices),-summary.MinimumObservedClearance(indices)];
normalized=zeros(size(objective));
for j=1:size(objective,2)
    range=max(objective(:,j))-min(objective(:,j));
    if range>1e-12,normalized(:,j)=(objective(:,j)-min(objective(:,j)))/range;end
end
weights=[0.40 0.20 0.15 0.15 0.10];
score=normalized*weights.';summary.TrainingScore(indices)=score;
pareto=true(numel(indices),1);
for i=1:numel(indices)
    dominated=all(objective<=objective(i,:),2)&any(objective<objective(i,:),2);
    dominated(i)=false;if any(dominated),pareto(i)=false;end
end
summary.Pareto(indices)=pareto;
[~,bestLocal]=min(score);bestIndex=indices(bestLocal);
summary.SelectedByTraining(bestIndex)=true;
validation=find(strcmp(summary.Split,'VALIDATION')& ...
    summary.BezierFraction==summary.BezierFraction(bestIndex)& ...
    summary.TimeSlack==summary.TimeSlack(bestIndex));
summary.SelectedByTraining(validation)=true;
end

function energy=referenceCurvatureEnergy(reference)
ds=hypot(diff(reference.x),diff(reference.y));
dtheta=wrapAngle(diff(reference.theta));moving=ds>1e-8;
curvature=zeros(size(ds));curvature(moving)=dtheta(moving)./ds(moving);
energy=sum(curvature(moving).^2.*ds(moving));
if any(~moving&abs(dtheta)>1e-6),energy=inf;end
end
function metrics=emptyMetrics()
metrics=struct('CompletionTime',nan,'PositionRMSE',nan, ...
    'MinimumClearance',nan,'Jomega',nan,'NumberOfFullStops',nan);
end
function value=ternary(condition,a,b)
if condition,value=a;else,value=b;end
end
