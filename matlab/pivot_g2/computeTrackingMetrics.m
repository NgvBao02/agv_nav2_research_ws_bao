function metrics = computeTrackingMetrics(simulation, reference, decisions, config)
%COMPUTETRACKINGMETRICS Tinh cac chi so chuyen dong va an toan.
dt = config.dt;
actualPathLength = sum(hypot(diff(simulation.x),diff(simulation.y)));
referencePathLength = sum(hypot(diff(reference.x),diff(reference.y)));
stationary = abs(simulation.v) < config.controller.stationaryThreshold;
movingSeen = cumsum(~stationary)>0;
movingAfter = flipud(cumsum(flipud(~stationary))>0);
internalStationary = stationary & movingSeen & movingAfter;
starts = internalStationary & [true;~internalStationary(1:end-1)];
numberOfStops = sum(starts);
totalStoppedTime = sum(internalStationary)*dt;
if isempty(decisions)
    totalPivotAngle = 0;
    arcCorners = 0;
    pivotCorners = 0;
else
    isArc = strcmp({decisions.selectedType},'ARC');
    arcCorners = sum(isArc);
    pivotCorners = sum(~isArc);
    pivotDecisions = decisions(~isArc);
    if isempty(pivotDecisions)
        totalPivotAngle = 0;
    else
        totalPivotAngle = sum(arrayfun(@(d)abs(d.corner.turnAngle), ...
            pivotDecisions));
    end
end
collisionCount = sum(simulation.collision);
metrics = struct();
metrics.ReferencePathLength = referencePathLength;
metrics.ActualPathLength = actualPathLength;
metrics.CompletionTime = simulation.time(end);
metrics.NumberOfFullStops = numberOfStops;
metrics.TotalStoppedTime = totalStoppedTime;
metrics.TotalPivotAngle = totalPivotAngle;
metrics.AverageLinearVelocity = mean(abs(simulation.v));
metrics.PositionRMSE = sqrt(mean(simulation.positionError.^2));
metrics.HeadingRMSE = sqrt(mean(simulation.headingError.^2));
metrics.MinimumClearance = min(simulation.clearance);
metrics.Jv = sum(abs(diff(simulation.v)));
metrics.Jomega = sum(abs(diff(simulation.omega)));
metrics.MaximumLeftWheelVelocity = max(abs(simulation.leftWheelVelocity));
metrics.MaximumRightWheelVelocity = max(abs(simulation.rightWheelVelocity));
metrics.LimitViolationCount = simulation.limitViolationCount;
metrics.CollisionCount = collisionCount;
metrics.NumberOfArcCorners = arcCorners;
metrics.NumberOfPivotCorners = pivotCorners;
metrics.Success = simulation.completed && collisionCount==0 && ...
    metrics.MinimumClearance >= config.robot.clearanceSafe-1e-6 && ...
    metrics.LimitViolationCount==0;
metrics.TaskCompletionRate = double(metrics.Success);
metrics.TimePerMeter = metrics.CompletionTime/max(actualPathLength,eps);
metrics.StopsPerCorner = numberOfStops/max(numel(decisions),1);
metrics.JvPerMeter = metrics.Jv/max(actualPathLength,eps);
metrics.JomegaPerMeter = metrics.Jomega/max(actualPathLength,eps);
metrics.ArcSelectionRate = arcCorners/max(numel(decisions),1);
end
