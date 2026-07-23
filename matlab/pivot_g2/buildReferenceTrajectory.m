function reference = buildReferenceTrajectory(reducedPath, decisions, config)
%BUILDREFERENCETRAJECTORY Ghep doan thang, cung tron va quay tai cho.
validateattributes(reducedPath, {'numeric'}, {'2d','ncols',2,'finite'});
if size(reducedPath,1) < 2
    error('Duong tinh gon phai co it nhat hai waypoint.');
end

x = zeros(0,1); y = zeros(0,1); theta = zeros(0,1);
mode = cell(0,1); radius = zeros(0,1); speedLimit = zeros(0,1);
currentPoint = reducedPath(1,:);
initialDirection = reducedPath(2,:)-reducedPath(1,:);
initialHeading = atan2(initialDirection(2),initialDirection(1));
appendData(currentPoint(1),currentPoint(2),initialHeading, ...
    {'STRAIGHT'},inf,config.robot.maxLinearSpeed);

for i = 1:numel(decisions)
    decision = decisions(i);
    corner = decision.corner;
    if strcmp(decision.selectedType,'ARC')
        maneuverPoses = decision.selectedPoses;
        entryPoint = maneuverPoses(1,1:2);
        exitPoint = maneuverPoses(end,1:2);
    else
        entryPoint = corner.vertex;
        exitPoint = corner.vertex;
        maneuverPoses = decision.pivot.poses;
    end
    appendStraight(currentPoint,entryPoint,corner.inDirection);
    if strcmp(decision.selectedType,'ARC')
        if strcmp(corner.turnDirection,'LEFT')
            maneuverMode = 'ARC_LEFT';
        else
            maneuverMode = 'ARC_RIGHT';
        end
        selectedCandidate = decision.arcCandidates(decision.bestArcIndex);
        if isfield(selectedCandidate,'radiusProfile') && ...
                numel(selectedCandidate.radiusProfile)==size(maneuverPoses,1)
            maneuverRadius=selectedCandidate.radiusProfile(:);
        else
            maneuverRadius=repmat(decision.selectedRadius,size(maneuverPoses,1),1);
        end
        if isfield(selectedCandidate,'speedLimitProfile') && ...
                numel(selectedCandidate.speedLimitProfile)==size(maneuverPoses,1)
            maneuverSpeedLimit=selectedCandidate.speedLimitProfile(:);
        else
            maneuverSpeedLimit=repmat(selectedCandidate.vArc,size(maneuverPoses,1),1);
        end
        appendData(maneuverPoses(:,1),maneuverPoses(:,2), ...
            maneuverPoses(:,3),repmat({maneuverMode},size(maneuverPoses,1),1), ...
            maneuverRadius,maneuverSpeedLimit);
    else
        if strcmp(corner.turnDirection,'LEFT')
            maneuverMode = 'PIVOT_LEFT';
        else
            maneuverMode = 'PIVOT_RIGHT';
        end
        appendData(maneuverPoses(:,1),maneuverPoses(:,2), ...
            maneuverPoses(:,3),repmat({maneuverMode},size(maneuverPoses,1),1), ...
            zeros(size(maneuverPoses,1),1),zeros(size(maneuverPoses,1),1));
    end
    currentPoint = exitPoint;
end

finalDirection = reducedPath(end,:)-reducedPath(end-1,:);
appendStraight(currentPoint,reducedPath(end,:),finalDirection/norm(finalDirection));

% Chuan hoa goc theo nhanh lien tuc de noi suy va tinh profile.
theta = unwrap(theta);
reference = struct('x',x,'y',y,'theta',theta,'v',zeros(size(x)), ...
    'omega',zeros(size(x)),'mode',{mode},'radius',radius, ...
    'speedLimit',speedLimit,'time',zeros(size(x)), ...
    'linearAcceleration',zeros(size(x)), ...
    'angularAcceleration',zeros(size(x)));

    function appendStraight(pointA,pointB,direction)
        distance = norm(pointB-pointA);
        if distance < config.numericTolerance
            return;
        end
        count = max(2,ceil(distance/config.straightSampleSpacing)+1);
        fraction = linspace(0,1,count).';
        points = pointA + fraction.*(pointB-pointA);
        heading = atan2(direction(2),direction(1));
        appendData(points(:,1),points(:,2),repmat(heading,count,1), ...
            repmat({'STRAIGHT'},count,1),repmat(inf,count,1), ...
            repmat(config.robot.maxLinearSpeed,count,1));
    end

    function appendData(newX,newY,newTheta,newMode,newRadius,newSpeedLimit)
        newX = newX(:); newY = newY(:); newTheta = newTheta(:);
        newRadius = newRadius(:); newSpeedLimit = newSpeedLimit(:);
        if ~iscell(newMode), newMode = cellstr(newMode); end
        if ~isempty(x) && ~isempty(newX) && hypot(newX(1)-x(end),newY(1)-y(end)) < 1e-12 ...
                && abs(wrapAngle(newTheta(1)-theta(end))) < 1e-12
            newX(1)=[]; newY(1)=[]; newTheta(1)=[]; newMode(1)=[];
            newRadius(1)=[]; newSpeedLimit(1)=[];
        end
        if isempty(newX), return; end
        if ~isempty(theta)
            newTheta = newTheta + 2*pi*round((theta(end)-newTheta(1))/(2*pi));
        end
        x = [x;newX]; y = [y;newY]; theta = [theta;newTheta]; %#ok<AGROW>
        mode = [mode;newMode(:)]; radius = [radius;newRadius]; %#ok<AGROW>
        speedLimit = [speedLimit;newSpeedLimit]; %#ok<AGROW>
    end
end
