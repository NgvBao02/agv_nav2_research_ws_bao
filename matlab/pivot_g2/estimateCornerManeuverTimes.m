function [pivot,candidates] = estimateCornerManeuverTimes( ...
        corner,pivot,candidates,config)
%ESTIMATECORNERMANEUVERTIMES So sanh Pivot/G2 tren cung entry-exit window.
% Moi ung vien gom ca hai doan thang bu tu bien cua so toi transition. Pivot
% dung hai doan thang day du, dung tai dinh va quay. Cung mot time parameterizer
% va cung boundary-speed target duoc dung cho tat ca phuong an.
windowDistance=config.timeComparison.windowFraction* ...
    min(corner.lengthBefore,corner.lengthAfter);
boundarySpeed=min(config.robot.maxLinearSpeed, ...
    max(0,config.timeComparison.boundarySpeed));
robot=config.robot;

% Neu cua so ngan, toc do tai bien phai giam truoc do. Phan thoi gian chuyen
% tiep o ngoai cua so duoc cong vao de khong uu ai phuong an can slowdown som.
pivotEntrySpeed=min(boundarySpeed,sqrt(2*robot.maxLinearDeceleration*windowDistance));
pivotExitSpeed=min(boundarySpeed,sqrt(2*robot.maxLinearAcceleration*windowDistance));
entryPenalty=(boundarySpeed-pivotEntrySpeed)/robot.maxLinearDeceleration;
exitPenalty=(boundarySpeed-pivotExitSpeed)/robot.maxLinearAcceleration;
approachTime=minimumTranslationTime(windowDistance,pivotEntrySpeed,0,robot);
departureTime=minimumTranslationTime(windowDistance,0,pivotExitSpeed,robot);
rotationTime=minimumRotationTime(corner.turnAngle,robot);
pivot.legacyPredictedTime=pivot.predictedTime;
pivot.predictedTime=entryPenalty+approachTime+rotationTime+ ...
    departureTime+exitPenalty;
pivot.comparisonWindowDistance=windowDistance;
pivot.timeDetails=struct('model','COMMON_WINDOW_TIME_PARAMETERIZED', ...
    'boundarySpeedTarget',boundarySpeed,'entryBoundarySpeed',pivotEntrySpeed, ...
    'exitBoundarySpeed',pivotExitSpeed,'entryTransitionTime',entryPenalty, ...
    'approachTime',approachTime,'rotationTime',rotationTime, ...
    'departureTime',departureTime,'exitTransitionTime',exitPenalty, ...
    'windowDistance',windowDistance);

outerEntry=corner.vertex-windowDistance*corner.inDirection;
outerExit=corner.vertex+windowDistance*corner.outDirection;
for k=1:numel(candidates)
    item=candidates(k);item.comparisonWindowDistance=windowDistance;
    if ~item.valid,candidates(k)=item;continue;end
    trimDistance=item.arc.tangentDistance;
    if trimDistance>windowDistance+config.numericTolerance
        item.valid=false;item.reason='Transition nam ngoai common time window.';
        candidates(k)=item;continue;
    end
    curvePoints=item.poses(:,1:2);
    if isfield(item,'curvatureProfile')&& ...
            numel(item.curvatureProfile)==size(curvePoints,1)
        curveCurvature=item.curvatureProfile(:);
        curveCaps=item.speedLimitProfile(:);
    else
        curveCurvature=sign(corner.turnAngle)/item.radius* ...
            ones(size(curvePoints,1),1);
        curveCaps=item.vArc*ones(size(curvePoints,1),1);
    end
    prefix=sampleLine(outerEntry,curvePoints(1,:),config.straightSampleSpacing);
    suffix=sampleLine(curvePoints(end,:),outerExit,config.straightSampleSpacing);
    points=[prefix(1:end-1,:);curvePoints;suffix(2:end,:)];
    curvature=[zeros(size(prefix,1)-1,1);curveCurvature; ...
        zeros(size(suffix,1)-1,1)];
    speedLimit=[robot.maxLinearSpeed*ones(size(prefix,1)-1,1);curveCaps; ...
        robot.maxLinearSpeed*ones(size(suffix,1)-1,1)];
    profile=timeParameterizeMovingPath(points,curvature,speedLimit,robot, ...
        boundarySpeed,boundarySpeed);
    if ~profile.valid
        item.valid=false;item.reason=['Time profile khong kha thi: ' profile.reason];
        item.timeProfile=profile;candidates(k)=item;continue;
    end
    profile.windowTraversalTime=profile.totalTime;
    profile.entryTransitionTime=max(0,boundarySpeed-profile.linearSpeed(1))/ ...
        robot.maxLinearDeceleration;
    profile.exitTransitionTime=max(0,boundarySpeed-profile.linearSpeed(end))/ ...
        robot.maxLinearAcceleration;
    profile.totalTimeWithBoundaryTransitions=profile.windowTraversalTime+ ...
        profile.entryTransitionTime+profile.exitTransitionTime;
    item.predictedTime=profile.totalTimeWithBoundaryTransitions;
    item.timeProfile=profile;
    item.comparisonWindowDistance=windowDistance;
    [~,peak]=max(abs(profile.angularSpeed));item.omega=profile.angularSpeed(peak);
    left=profile.linearSpeed-robot.wheelBase*profile.angularSpeed/2;
    right=profile.linearSpeed+robot.wheelBase*profile.angularSpeed/2;
    item.leftWheelSpeed=max(abs(left));item.rightWheelSpeed=max(abs(right));
    candidates(k)=item;
end
end

function points=sampleLine(first,last,spacing)
distance=norm(last-first);
if distance<=1e-12,points=first;return;end
count=max(2,ceil(distance/spacing)+1);
fraction=linspace(0,1,count).';
points=first+fraction.*(last-first);
end
