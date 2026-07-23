function arc = generateTangentArc(corner,radius,maxSpacing)
%GENERATETANGENTARC Sinh cung tron tiep tuyen cho goc bat ky trong (0,pi).
validateattributes(radius,{'numeric'},{'scalar','positive','finite'});
validateattributes(maxSpacing,{'numeric'},{'scalar','positive','finite'});
angleMagnitude=abs(corner.turnAngle);
if angleMagnitude<1e-6 || angleMagnitude>=pi-1e-6
    error('Goc re phai nam trong khoang (0,pi).');
end
turnSign=sign(corner.turnAngle);
tangentDistance=radius*tan(angleMagnitude/2);
startPoint=corner.vertex-tangentDistance*corner.inDirection;
endPoint=corner.vertex+tangentDistance*corner.outDirection;
leftNormal=[-corner.inDirection(2),corner.inDirection(1)];
center=startPoint+turnSign*radius*leftNormal;
startRadialAngle=atan2(startPoint(2)-center(2),startPoint(1)-center(1));
arcLength=radius*angleMagnitude;
sampleCount=max(2,ceil(arcLength/maxSpacing)+1);
radialAngle=startRadialAngle+linspace(0,corner.turnAngle,sampleCount).';
x=center(1)+radius*cos(radialAngle);
y=center(2)+radius*sin(radialAngle);
theta=unwrap(radialAngle+turnSign*pi/2);
% Ghim diem cuoi hinh hoc de loai sai so lam tron luong giac.
x([1 end])=[startPoint(1);endPoint(1)];
y([1 end])=[startPoint(2);endPoint(2)];
arc=struct('radius',radius,'center',center,'startPoint',startPoint, ...
    'endPoint',endPoint,'poses',[x y theta],'length',arcLength, ...
    'turnDirection',corner.turnDirection,'tangentDistance',tangentDistance);
end
