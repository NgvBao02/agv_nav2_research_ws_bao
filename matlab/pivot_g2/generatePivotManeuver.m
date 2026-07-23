function pivot = generatePivotManeuver(corner, map, config)
%GENERATEPIVOTMANEUVER Sinh pose quay tai cho va kiem tra vung quet.
headingIn = atan2(corner.inDirection(2),corner.inDirection(1));
sampleCount = max(2,ceil(abs(corner.turnAngle)/config.pivotAngleStep)+1);
theta = headingIn + linspace(0,corner.turnAngle,sampleCount).';
poses = [repmat(corner.vertex,sampleCount,1),theta];
timer=tic;
safety=evaluatePoseSequenceSafety(poses,map,config);
footprintTime=toc(timer);
minimumClearance=safety.minimumClearance;
collision=~safety.safe;
[predictedTime,timeDetails] = estimatePivotTime(corner.turnAngle, ...
    config.robot.maxLinearSpeed,config.robot.maxLinearSpeed,config.robot);
pivot = struct('poses',poses,'valid',~collision, ...
    'minimumClearance',minimumClearance,'predictedTime',predictedTime, ...
    'legacyPredictedTime',predictedTime,'comparisonWindowDistance',nan, ...
    'timeDetails',timeDetails,'footprintCheckTime',footprintTime, ...
    'reason','');
if pivot.valid
    pivot.reason = 'Hop le.';
else
    pivot.reason = 'Vung quet footprint khi quay khong an toan.';
end
end
