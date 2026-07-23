function duration = minimumRotationTime(angle,robot)
%MINIMUMROTATIONTIME Profile tam giac/hinh thang cho quay tai cho.
magnitude=abs(angle);
if ~isfinite(magnitude)||robot.maxAngularSpeed<=0|| ...
        robot.maxAngularAcceleration<=0
    duration=inf;return;
end
critical=robot.maxAngularSpeed^2/robot.maxAngularAcceleration;
if magnitude<=critical
    duration=2*sqrt(magnitude/robot.maxAngularAcceleration);
else
    duration=2*robot.maxAngularSpeed/robot.maxAngularAcceleration+ ...
        (magnitude-critical)/robot.maxAngularSpeed;
end
end
