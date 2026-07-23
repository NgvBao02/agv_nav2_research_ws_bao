function [totalTime, details] = estimateArcTime(radius, turnAngle, vBefore, vAfter, robot)
%ESTIMATEARCTIME Tinh toc do cung tu gioi han than, omega va hai banh.
if radius <= 0
    error('Ban kinh cung phai duong.');
end
turnMagnitude = abs(turnAngle);
if radius < robot.wheelBase/2-1e-12
    totalTime = inf;
    details = invalidDetails('Banh trong phai quay nguoc.');
    return;
end
wheelLimitedSpeed = robot.maxWheelSpeed / ...
    (1 + robot.wheelBase/(2*radius));
vArc = min([robot.maxLinearSpeed, ...
    radius*robot.maxAngularSpeed,wheelLimitedSpeed]);
if vArc <= 0
    totalTime = inf;
    details = invalidDetails('Khong ton tai toc do cung duong.');
    return;
end
omega = sign(turnAngle)*vArc/radius;
rightWheel = vArc + robot.wheelBase*omega/2;
leftWheel = vArc - robot.wheelBase*omega/2;
if max(abs([rightWheel leftWheel])) > robot.maxWheelSpeed+1e-10
    totalTime = inf;
    details = invalidDetails('Toc do banh vuot gioi han.');
    return;
end
if min([rightWheel leftWheel]) < -1e-10
    totalTime = inf;
    details = invalidDetails('Banh trong quay nguoc.');
    return;
end
entryTime = transitionTime(vBefore,vArc,robot);
exitTime = transitionTime(vArc,vAfter,robot);
arcLength = radius*turnMagnitude;
cruiseTime = arcLength/vArc;
totalTime = entryTime + cruiseTime + exitTime;
details = struct('valid',true,'reason','Hop le.','vArc',vArc, ...
    'omega',omega,'leftWheelSpeed',leftWheel, ...
    'rightWheelSpeed',rightWheel,'entryTransitionTime',entryTime, ...
    'arcCruiseTime',cruiseTime,'exitTransitionTime',exitTime);
end

function time = transitionTime(v0,v1,robot)
if v1 >= v0
    time = (v1-v0)/robot.maxLinearAcceleration;
else
    time = (v0-v1)/robot.maxLinearDeceleration;
end
end

function details = invalidDetails(reason)
details = struct('valid',false,'reason',reason,'vArc',nan,'omega',nan, ...
    'leftWheelSpeed',nan,'rightWheelSpeed',nan, ...
    'entryTransitionTime',nan,'arcCruiseTime',nan, ...
    'exitTransitionTime',nan);
end
