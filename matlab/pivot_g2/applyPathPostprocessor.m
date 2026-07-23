function [reference,processedPath,decisions,info,method] = ...
        applyPathPostprocessor(methodName,inputPath,planningOccupancy, ...
        map,config,comparison)
%APPLYPATHPOSTPROCESSOR API chung cua sau bo hau xu ly.
method=methodMetadata(methodName);methodName=method.name;
timer=tic;decisions=struct([]);specific=struct();
switch methodName
    case 'NO_SMOOTHER'
        processedPath=inputPath;
        validation=validatePostprocessedPath(processedPath,map,config, ...
            comparison.reference.sampleSpacing);
        specific=struct('plugin','NONE','implementation','BASELINE', ...
            'iterations',0,'maximumDisplacement',0,'coreTime',0, ...
            'validation',validation, ...
            'message','Khong thay doi toa do XY.');
        reference=buildContinuousReferenceFromPath(processedPath,config,comparison);
    case 'NAV2_SIMPLE'
        [processedPath,specific]=nav2SimpleSmootherEquivalent(inputPath, ...
            planningOccupancy,map,config,comparison);
        reference=buildContinuousReferenceFromPath(processedPath,config,comparison);
    case 'NAV2_SAVITZKY_GOLAY'
        [processedPath,specific]=nav2SavitzkyGolaySmootherEquivalent( ...
            inputPath,map,config,comparison);
        reference=buildContinuousReferenceFromPath(processedPath,config,comparison);
    case 'NAV2_CONSTRAINED'
        [processedPath,specific]=nav2ConstrainedSmootherEquivalent(inputPath, ...
            planningOccupancy,map,config,comparison);
        reference=buildContinuousReferenceFromPath(processedPath,config,comparison);
    case {'FIXED_RADIUS_ARC','PROPOSED_PIVOT_ARC'}
        arcInputPath=inputPath;
        if isfield(comparison,'arcPreprocessing')&& ...
                comparison.arcPreprocessing.lineOfSightPruning
            arcInputPath=prunePathLineOfSight(inputPath,planningOccupancy,map);
        end
        reduced=removeCollinearPoints(arcInputPath);corners=detectCorners(reduced);
        if strcmp(methodName,'FIXED_RADIUS_ARC'),selector='FIXED_RADIUS';
        else,selector='ADAPTIVE_PIVOT_OR_ARC';end
        [decisions,timing]=chooseManeuversForMethod(corners,map,config,selector);
        if ~isempty(decisions)&&~all([decisions.valid])
            error('%s khong tao duoc maneuver an toan cho tat ca goc.',methodName);
        end
        reference=buildReferenceTrajectory(reduced,decisions,config);
        refinementInfo=struct('enabled',false,'applied',false, ...
            'maximumDisplacement',0,'iterations',0);
        if strcmp(methodName,'PROPOSED_PIVOT_ARC')&& ...
                isfield(comparison,'proposedRefinement')&& ...
                comparison.proposedRefinement.enabled
            refinementTimer=tic;
            localComparison.commonSmoother=comparison.proposedRefinement;
            [reference,refinementInfo]=smoothReferenceTrajectoryCommon( ...
                reference,map,config,localComparison);
            refinementCoreTime=toc(refinementTimer);
        else
            refinementCoreTime=0;
        end
        processedPath=[reference.x reference.y];
        validation=validateReferenceSafety(reference,map,config);
        if ~validation.safe,error('%s tao reference khong an toan.',methodName);end
        specific=timing;specific.plugin=method.plugin;
        specific.implementation=method.implementation;
        specific.iterations=refinementInfo.iterations;
        specific.coreTime=timing.cornerOptimizationTime+refinementCoreTime;
        specific.maximumDisplacement= ...
            maximumDistanceToInput(processedPath,inputPath);
        specific.validation=validation;specific.message='OK';
        specific.refinementEnabled=refinementInfo.enabled;
        specific.refinementApplied=refinementInfo.applied;
        specific.refinementMaximumDisplacement= ...
            refinementInfo.maximumDisplacement;
        specific.g2GuaranteePreserved=~refinementInfo.applied;
        specific.preprocessedPointCount=size(arcInputPath,1);
        if size(arcInputPath,1)<size(inputPath,1)
            specific.preprocessing='VISIBILITY_PRUNING';
        else
            specific.preprocessing='COLLINEAR_REDUCTION_ONLY';
        end
        reference=generateSpeedProfile(reference,config);
    otherwise
        error('Bo hau xu ly khong hop le: %s',methodName);
end
if ~isfield(specific,'validation')||~specific.validation.safe
    error('%s khong tao duoc path footprint-safe.',methodName);
end
info=specific;info.name=method.name;info.displayName=method.displayName;
if ~isfield(info,'coreTime'),info.coreTime=0;end
if ~isfield(info,'preprocessing'),info.preprocessing='NONE';end
info.postprocessTime=toc(timer);info.inputPathUnchanged=true;
info.outputPointCount=size(processedPath,1);
end

function method=methodMetadata(name)
name=upper(char(name));
switch name
    case 'NO_SMOOTHER'
        method=make(name,'No smoother','NONE','BASELINE');
    case 'NAV2_SIMPLE'
        method=make(name,'Nav2 Simple','nav2_smoother::SimpleSmoother', ...
            'MATLAB_EQUIVALENT');
    case 'NAV2_SAVITZKY_GOLAY'
        method=make(name,'Nav2 Savitzky-Golay', ...
            'nav2_smoother::SavitzkyGolaySmoother','MATLAB_EQUIVALENT');
    case 'NAV2_CONSTRAINED'
        method=make(name,'Nav2 Constrained', ...
            'nav2_constrained_smoother::ConstrainedSmoother', ...
            'MATLAB_EQUIVALENT_OBJECTIVE');
    case 'FIXED_RADIUS_ARC'
        method=make(name,'Fixed-radius arc','CUSTOM_FIXED_RADIUS', ...
            'MATLAB_BASELINE');
    case 'PROPOSED_PIVOT_ARC'
        method=make(name,'Proposed pivot-arc','CUSTOM_ADAPTIVE_PIVOT_ARC', ...
            'PROPOSED');
    otherwise,error('Khong biet method %s.',name);
end
end
function value=make(name,displayName,plugin,implementation)
value=struct('name',name,'displayName',displayName,'plugin',plugin, ...
    'implementation',implementation);
end
function distance=maximumDistanceToInput(output,input)
% Khoang cach tu diem output toi polyline input, khong chi toi waypoint.
distance=0;
for i=1:size(output,1)
    nearest=inf;
    for j=1:size(input,1)-1
        segment=input(j+1,:)-input(j,:);
        fraction=dot(output(i,:)-input(j,:),segment)/max(dot(segment,segment),eps);
        fraction=min(1,max(0,fraction));
        projection=input(j,:)+fraction*segment;
        nearest=min(nearest,norm(output(i,:)-projection));
    end
    distance=max(distance,nearest);
end
end
