function [totalTime, details] = estimatePivotTime(turnAngle, vBefore, vAfter, robot)
%ESTIMATEPIVOTTIME Uoc luong thoi gian giam toc, quay va tang toc.
validateattributes(turnAngle, {'numeric'}, {'scalar','finite'});
angle = abs(turnAngle);
vBefore = max(0,min(robot.maxLinearSpeed,vBefore));
vAfter = max(0,min(robot.maxLinearSpeed,vAfter));
decelerationTime = vBefore / robot.maxLinearDeceleration;
accelerationTime = vAfter / robot.maxLinearAcceleration;
criticalAngle = robot.maxAngularSpeed^2 / robot.maxAngularAcceleration;
if angle <= criticalAngle
    rotationTime = 2*sqrt(angle/robot.maxAngularAcceleration);
    angularProfile = 'TRIANGULAR';
else
    rotationTime = 2*robot.maxAngularSpeed/robot.maxAngularAcceleration + ...
        (angle-criticalAngle)/robot.maxAngularSpeed;
    angularProfile = 'TRAPEZOIDAL';
end
totalTime = decelerationTime + rotationTime + accelerationTime;
details = struct('decelerationTime',decelerationTime, ...
    'rotationTime',rotationTime,'accelerationTime',accelerationTime, ...
    'angularProfile',angularProfile);
end
