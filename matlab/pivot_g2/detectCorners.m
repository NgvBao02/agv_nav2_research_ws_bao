function corners = detectCorners(reducedPath)
%DETECTCORNERS Phat hien va mo ta cac goc tu ba waypoint lien tiep.
validateattributes(reducedPath, {'numeric'}, {'2d','ncols',2,'finite'});
template = struct('pathIndex',0,'vertex',[0 0],'inDirection',[0 0], ...
    'outDirection',[0 0],'turnAngle',0,'turnDirection','STRAIGHT', ...
    'lengthBefore',0,'lengthAfter',0);
corners = repmat(template,0,1);
for i = 2:size(reducedPath,1)-1
    inVector = reducedPath(i,:) - reducedPath(i-1,:);
    outVector = reducedPath(i+1,:) - reducedPath(i,:);
    lengthBefore = norm(inVector);
    lengthAfter = norm(outVector);
    if lengthBefore < eps || lengthAfter < eps
        continue;
    end
    inDirection = inVector / lengthBefore;
    outDirection = outVector / lengthAfter;
    signedAngle = atan2(inDirection(1)*outDirection(2) - ...
        inDirection(2)*outDirection(1), dot(inDirection,outDirection));
    if abs(signedAngle) < 1e-10
        turnDirection = 'STRAIGHT';
    elseif signedAngle > 0
        turnDirection = 'LEFT';
    else
        turnDirection = 'RIGHT';
    end
    if strcmp(turnDirection,'STRAIGHT')
        continue;
    end
    item = template;
    item.pathIndex = i;
    item.vertex = reducedPath(i,:);
    item.inDirection = inDirection;
    item.outDirection = outDirection;
    item.turnAngle = signedAngle;
    item.turnDirection = turnDirection;
    item.lengthBefore = lengthBefore;
    item.lengthAfter = lengthAfter;
    corners(end+1,1) = item; %#ok<AGROW>
end
end
