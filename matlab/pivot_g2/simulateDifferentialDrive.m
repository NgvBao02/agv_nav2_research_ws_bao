function simulation = simulateDifferentialDrive(reference, map, config)
%SIMULATEDIFFERENTIALDRIVE Bam reference bang controller va mo hinh vi sai.
robot = config.robot;
controller = config.controller;
dt = config.dt;
maximumTime = 2*reference.time(end) + controller.extraSimulationTime;
maximumSteps = ceil(maximumTime/dt)+1;

fields = {'time','x','y','theta','v','omega','leftWheelVelocity', ...
    'rightWheelVelocity','linearAcceleration','angularAcceleration', ...
    'positionError','headingError','clearance', ...
    'collision','referenceIndex'};
for f = 1:numel(fields)
    logData.(fields{f}) = zeros(maximumSteps,1);
end
pose = [reference.x(1),reference.y(1),reference.theta(1)];
vPrevious = 0; omegaPrevious = 0;
targetIndex = min(2,numel(reference.x));
goalHoldSamples = 0;
requiredHoldSamples = max(1,ceil(controller.goalHoldTime/dt));
completed = false;
limitViolationCount = 0;

for k = 1:maximumSteps
    timeNow = (k-1)*dt;
    % Chieu pose len nhanh reference dang chuyen dong. Projection chi tim
    % ve phia truoc va khong duoc nhay qua mot cum pivot, nen tien do don
    % dieu nhung khong bi tre waypoint - nguyen nhan cua dao dong zig-zag.
    if targetIndex < numel(reference.x) && ...
            ~startsWith(reference.mode{targetIndex},'PIVOT')
        searchEnd = min(numel(reference.x), ...
            targetIndex+controller.progressSearchWindow);
        nextPivot=find(startsWith(reference.mode(targetIndex:searchEnd),'PIVOT'),1);
        if ~isempty(nextPivot)
            searchEnd = targetIndex+nextPivot-2;
        end
        if searchEnd >= targetIndex
            candidates = targetIndex:searchEnd;
            squaredDistance = (reference.x(candidates)-pose(1)).^2 + ...
                (reference.y(candidates)-pose(2)).^2;
            [~,nearestLocal] = min(squaredDistance);
            targetIndex = max(targetIndex,candidates(nearestLocal));
        end
    end
    % Tang chi so theo sai so hinh hoc, khong ep robot bam theo dong ho.
    % Nho vay qu quy dao thuc luon do controller sinh ra va khong nhay mau.
    advancing = true;
    while advancing && targetIndex < numel(reference.x)
        targetMode = reference.mode{targetIndex};
        targetPositionError = hypot(reference.x(targetIndex)-pose(1), ...
            reference.y(targetIndex)-pose(2));
        targetHeadingError = abs(wrapAngle(reference.theta(targetIndex)-pose(3)));
        if startsWith(targetMode,'PIVOT')
            advancing = targetHeadingError < controller.pivotAngleTolerance;
        else
            tolerance = controller.pointTolerance;
            if targetIndex < numel(reference.x) && ...
                    startsWith(reference.mode{targetIndex+1},'PIVOT')
                tolerance = controller.cornerPositionTolerance;
            end
            targetDirection = [cos(reference.theta(targetIndex)), ...
                sin(reference.theta(targetIndex))];
            targetDelta = [reference.x(targetIndex)-pose(1), ...
                reference.y(targetIndex)-pose(2)];
            passedTarget = dot(targetDelta,targetDirection) < -0.005;
            advancing = targetPositionError < tolerance || passedTarget;
        end
        if advancing
            targetIndex = targetIndex+1;
        end
    end
    controlIndex = targetIndex;
    if ~startsWith(reference.mode{targetIndex},'PIVOT')
        lookAheadDistance = controller.minimumLookAheadDistance + ...
            abs(reference.v(targetIndex))*controller.lookAheadTime;
        lookAheadDistance = min(controller.maximumLookAheadDistance, ...
            max(controller.minimumLookAheadDistance,lookAheadDistance));
        if startsWith(reference.mode{targetIndex},'ARC') && ...
                isfinite(reference.radius(targetIndex))
            arcLookAhead=max(controller.minimumArcLookAheadDistance, ...
                controller.arcLookAheadRadiusFraction*reference.radius(targetIndex));
            lookAheadDistance=min(lookAheadDistance,arcLookAhead);
        end
        accumulatedDistance = 0;
        while controlIndex < numel(reference.x) && ...
                ~startsWith(reference.mode{controlIndex+1},'PIVOT') && ...
                sameTrackingMode(reference.mode{controlIndex+1}, ...
                    reference.mode{targetIndex}) && ...
                accumulatedDistance < lookAheadDistance
            accumulatedDistance = accumulatedDistance + hypot( ...
                reference.x(controlIndex+1)-reference.x(controlIndex), ...
                reference.y(controlIndex+1)-reference.y(controlIndex));
            controlIndex = controlIndex+1;
        end
    end
    target = [reference.x(controlIndex),reference.y(controlIndex), ...
        reference.theta(controlIndex)];
    positionDelta = target(1:2)-pose(1:2);
    c = cos(pose(3)); s = sin(pose(3));
    trackingDelta=[reference.x(targetIndex)-pose(1), ...
        reference.y(targetIndex)-pose(2)];
    trackingBodyError = [c s;-s c]*trackingDelta.';
    controlBodyError = [c s;-s c]*positionDelta.';
    headingError = wrapAngle(reference.theta(targetIndex)-pose(3));
    mode = reference.mode{controlIndex};
    finalDistance = norm([reference.x(end)-pose(1),reference.y(end)-pose(2)]);
    isFinalTarget = targetIndex == numel(reference.x);
    isPrePivot = targetIndex < numel(reference.x) && ...
        ~startsWith(reference.mode{targetIndex},'PIVOT') && ...
        startsWith(reference.mode{targetIndex+1},'PIVOT');
    isFinalAlignment = isFinalTarget && ...
        finalDistance <= 0.5*controller.goalPositionTolerance;
    if isFinalAlignment
        vDesired = 0;
        omegaDesired = controller.kHeading*headingError;
    elseif isFinalTarget
        finalBearingError = wrapAngle(atan2(positionDelta(2),positionDelta(1))-pose(3));
        vDesired = controller.kLongitudinal*finalDistance*cos(finalBearingError);
        omegaDesired = controller.kHeading*finalBearingError;
    elseif isPrePivot
        % Mau cuoi truoc pivot co v_ref=0. Dung dieu khien pose-point de
        % xoa sai so ngang con lai; neu chi dung path feedback robot co the
        % dung cach tam pivot vai centimet va khong bao gio chuyen mode.
        pivotDistance=norm(trackingDelta);
        pivotBearingError=wrapAngle(atan2(trackingDelta(2),trackingDelta(1))-pose(3));
        vDesired=controller.kPivotPosition*pivotDistance*cos(pivotBearingError);
        omegaDesired=controller.kHeading*pivotBearingError;
    elseif startsWith(mode,'PIVOT')
        vDesired = 0;
        omegaDesired = reference.omega(targetIndex) + ...
            controller.kHeading*headingError;
    else
        if isfield(controller,'feedforwardAtProjection') && ...
                controller.feedforwardAtProjection
            feedforwardIndex=targetIndex;
        else
            feedforwardIndex=controlIndex;
        end
        feedforwardVelocity=reference.v(feedforwardIndex);
        vDesired = feedforwardVelocity*cos(headingError) + ...
            controller.kLongitudinal*controlBodyError(1);
        vDesired = vDesired*exp(-controller.headingSlowdown*abs(headingError));
        % Luat Kanayama/Frenet: feed-forward curvature tai phep chieu,
        % preview chi tao sai so doc. Khong chia cho khoang cach waypoint.
        omegaDesired = reference.omega(feedforwardIndex) + ...
            controller.kLateral*max(abs(feedforwardVelocity),0.03)* ...
            trackingBodyError(2) + controller.kHeading*sin(headingError);
    end
    vDesired = min(robot.maxLinearSpeed,max(-robot.maxLinearSpeed,vDesired));
    omegaDesired = min(robot.maxAngularSpeed,max(-robot.maxAngularSpeed,omegaDesired));
    if startsWith(mode,'ARC')
        vDesired=max(0,vDesired);
        omegaNoReverseDesired=2*vDesired/robot.wheelBase;
        omegaDesired=min(omegaNoReverseDesired, ...
            max(-omegaNoReverseDesired,omegaDesired));
    end

    if vDesired >= vPrevious
        vCommand = min(vDesired,vPrevious+robot.maxLinearAcceleration*dt);
    else
        vCommand = max(vDesired,vPrevious-robot.maxLinearDeceleration*dt);
    end
    omegaCommand = min(omegaPrevious+robot.maxAngularAcceleration*dt, ...
        max(omegaPrevious-robot.maxAngularAcceleration*dt,omegaDesired));
    if startsWith(mode,'ARC')
        vCommand = max(0,vCommand);
    end
    leftWheel = vCommand-robot.wheelBase*omegaCommand/2;
    rightWheel = vCommand+robot.wheelBase*omegaCommand/2;
    wheelScale = max(1,max(abs([leftWheel rightWheel]))/robot.maxWheelSpeed);
    vCommand = vCommand/wheelScale;
    omegaCommand = omegaCommand/wheelScale;
    leftWheel = vCommand-robot.wheelBase*omegaCommand/2;
    rightWheel = vCommand+robot.wheelBase*omegaCommand/2;

    collision = checkFootprintCollision(pose,map,robot,config.geometrySampleStep);
    clearance = computeMinimumClearance(pose,map,robot);
    logData.time(k)=timeNow; logData.x(k)=pose(1); logData.y(k)=pose(2);
    logData.theta(k)=pose(3); logData.v(k)=vCommand;
    logData.omega(k)=omegaCommand; logData.leftWheelVelocity(k)=leftWheel;
    logData.rightWheelVelocity(k)=rightWheel;
    logData.linearAcceleration(k)=(vCommand-vPrevious)/dt;
    logData.angularAcceleration(k)=(omegaCommand-omegaPrevious)/dt;
    progressDelta = [reference.x(targetIndex)-pose(1), ...
        reference.y(targetIndex)-pose(2)];
    progressHeadingError = wrapAngle(reference.theta(targetIndex)-pose(3));
    logData.positionError(k)=norm(progressDelta);
    logData.headingError(k)=progressHeadingError; logData.clearance(k)=clearance;
    logData.collision(k)=collision; logData.referenceIndex(k)=targetIndex;

    if abs(vCommand)>robot.maxLinearSpeed+1e-9 || ...
            abs(omegaCommand)>robot.maxAngularSpeed+1e-9 || ...
            max(abs([leftWheel rightWheel]))>robot.maxWheelSpeed+1e-9 || ...
            logData.linearAcceleration(k)>robot.maxLinearAcceleration+1e-9 || ...
            logData.linearAcceleration(k)<-robot.maxLinearDeceleration-1e-9 || ...
            abs(logData.angularAcceleration(k))>robot.maxAngularAcceleration+1e-9
        limitViolationCount = limitViolationCount+1;
    end

    goalPositionError = norm(pose(1:2)-[reference.x(end) reference.y(end)]);
    goalHeadingError = abs(wrapAngle(pose(3)-reference.theta(end)));
    if targetIndex == numel(reference.x) && ...
            goalPositionError <= controller.goalPositionTolerance && ...
            goalHeadingError <= controller.goalHeadingTolerance
        goalHoldSamples = goalHoldSamples+1;
        if goalHoldSamples >= requiredHoldSamples
            completed = true;
            finalIndex = k;
            break;
        end
    else
        goalHoldSamples = 0;
    end

    thetaMid = pose(3)+0.5*omegaCommand*dt;
    pose(1) = pose(1)+vCommand*cos(thetaMid)*dt;
    pose(2) = pose(2)+vCommand*sin(thetaMid)*dt;
    pose(3) = pose(3)+omegaCommand*dt;
    vPrevious = vCommand;
    omegaPrevious = omegaCommand;
    finalIndex = k;
end

for f = 1:numel(fields)
    logData.(fields{f}) = logData.(fields{f})(1:finalIndex);
end
simulation = logData;
simulation.completed = completed;
simulation.limitViolationCount = limitViolationCount;
simulation.finalPositionError = norm([simulation.x(end)-reference.x(end), ...
    simulation.y(end)-reference.y(end)]);
simulation.finalHeadingError = abs(wrapAngle(simulation.theta(end)-reference.theta(end)));
end

function same=sameTrackingMode(first,second)
% Khong de lookahead cat ngang bien STRAIGHT/ARC/PIVOT hoac doi chieu cung.
if startsWith(first,'PIVOT') || startsWith(second,'PIVOT')
    same=startsWith(first,'PIVOT')&&startsWith(second,'PIVOT');
elseif startsWith(first,'ARC') || startsWith(second,'ARC')
    same=strcmp(first,second);
else
    same=true;
end
end
