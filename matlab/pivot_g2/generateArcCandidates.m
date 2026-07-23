function candidates = generateArcCandidates(corner, radii, map, config)
%GENERATEARCCANDIDATES Sinh, danh gia footprint va uoc luong moi cung.
template = struct('radius',nan,'valid',false,'reason','', ...
    'arc',struct([]),'poses',zeros(0,3),'minimumClearance',nan, ...
    'predictedTime',inf,'vArc',nan,'omega',nan, ...
    'leftWheelSpeed',nan,'rightWheelSpeed',nan, ...
    'footprintCheckTime',0,'curveOnlyPredictedTime',inf, ...
    'comparisonWindowDistance',nan,'timeProfile',struct());
candidates = repmat(template,numel(radii),1);
for k = 1:numel(radii)
    radius = radii(k);
    item = template;
    item.radius = radius;
    angleMagnitude=abs(corner.turnAngle);
    if angleMagnitude<1e-6 || angleMagnitude>=pi-1e-6
        item.reason = 'Goc re khong nam trong khoang hop le (0, pi).';
        candidates(k) = item;
        continue;
    end
    maximumTangentDistance = config.maxCornerRadiusFraction * ...
        min(corner.lengthBefore,corner.lengthAfter);
    requiredTangentDistance=radius*tan(angleMagnitude/2);
    if requiredTangentDistance > maximumTangentDistance + config.numericTolerance
        item.reason = 'Doan thang lien ke khong du dai hoac de tranh cung chong lan.';
        candidates(k) = item;
        continue;
    end
    [time,timeDetails] = estimateArcTime(radius,corner.turnAngle, ...
        config.robot.maxLinearSpeed,config.robot.maxLinearSpeed,config.robot);
    if ~timeDetails.valid
        item.reason = timeDetails.reason;
        candidates(k) = item;
        continue;
    end
    arc = generateTangentArc(corner,radius,config.arcSampleSpacing);
    item.arc = arc;
    item.poses = arc.poses;
    item.predictedTime = time;
    item.curveOnlyPredictedTime = time;
    item.vArc = timeDetails.vArc;
    item.omega = timeDetails.omega;
    item.leftWheelSpeed = timeDetails.leftWheelSpeed;
    item.rightWheelSpeed = timeDetails.rightWheelSpeed;
    timer=tic;
    safety=evaluatePoseSequenceSafety(arc.poses,map,config);
    item.footprintCheckTime=toc(timer);
    item.minimumClearance=safety.minimumClearance;
    if ~safety.safe
        item.reason = sprintf('Footprint/clearance khong an toan (min %.3f m).', ...
            item.minimumClearance);
    else
        item.valid = true;
        item.reason = 'Hop le.';
    end
    candidates(k) = item;
end
end
