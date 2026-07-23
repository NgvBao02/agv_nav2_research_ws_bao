function arc = generateQuarterCircleArc(corner, radius, maxSpacing)
%GENERATEQUARTERCIRCLEARC Sinh cung tron tiep tuyen cho goc 90 do.
validateattributes(radius, {'numeric'}, {'scalar','positive','finite'});
validateattributes(maxSpacing, {'numeric'}, {'scalar','positive','finite'});
if abs(abs(corner.turnAngle)-pi/2) > 1e-6
    error('Phien ban nay chi sinh cung tron cho goc 90 do.');
end
turnSign = sign(corner.turnAngle);
startPoint = corner.vertex - radius*corner.inDirection;
endPoint = corner.vertex + radius*corner.outDirection;
leftNormal = [-corner.inDirection(2),corner.inDirection(1)];
center = startPoint + turnSign*radius*leftNormal;
startRadialAngle = atan2(startPoint(2)-center(2),startPoint(1)-center(1));
arcLength = radius*abs(corner.turnAngle);
sampleCount = max(2,ceil(arcLength/maxSpacing)+1);
radialAngle = startRadialAngle + linspace(0,corner.turnAngle,sampleCount).';
x = center(1) + radius*cos(radialAngle);
y = center(2) + radius*sin(radialAngle);
theta = radialAngle + turnSign*pi/2;
theta = unwrap(theta);
arc = struct('radius',radius,'center',center,'startPoint',startPoint, ...
    'endPoint',endPoint,'poses',[x y theta],'length',arcLength, ...
    'turnDirection',corner.turnDirection);
end
