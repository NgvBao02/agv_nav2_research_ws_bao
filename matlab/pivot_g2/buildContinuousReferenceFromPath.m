function reference = buildContinuousReferenceFromPath(path,config,comparison)
%BUILDCONTINUOUSREFERENCEFROMPATH Doi polyline thanh reference cong chung.
path=resamplePolyline(path,comparison.reference.sampleSpacing);
n=size(path,1);delta=zeros(n,2);
delta(1,:)=path(2,:)-path(1,:);delta(end,:)=path(end,:)-path(end-1,:);
if n>2,delta(2:end-1,:)=path(3:end,:)-path(1:end-2,:);end
theta=unwrap(atan2(delta(:,2),delta(:,1)));

% Loc nhe huong tiep tuyen chi de giam nhieu sai phan; khong doi XY.
window=max(1,round(comparison.reference.headingFilterWindow));
if window>1&&n>=window
    if mod(window,2)==0,window=window+1;end
    theta=movmean(theta,window,'Endpoints','shrink');
    theta(1)=atan2(path(2,2)-path(1,2),path(2,1)-path(1,1));
    theta(end)=theta(end)+2*pi*round((theta(end-1)-theta(end))/(2*pi));
end
ds=hypot(diff(path(:,1)),diff(path(:,2)));
sampleDs=[max(ds(1),eps);max((ds(1:end-1)+ds(2:end))/2,eps);max(ds(end),eps)];
dtheta=zeros(n,1);
dtheta(1)=wrapAngle(theta(2)-theta(1));
dtheta(end)=wrapAngle(theta(end)-theta(end-1));
if n>2,dtheta(2:end-1)=wrapAngle(theta(3:end)-theta(1:end-2))/2;end
curvature=dtheta./sampleDs;
curvature=max(-comparison.reference.maximumCurvature, ...
    min(comparison.reference.maximumCurvature,curvature));

mode=repmat({'STRAIGHT'},n,1);radius=inf(n,1);
arcMask=abs(curvature)>=comparison.reference.straightCurvatureThreshold;
left=arcMask&curvature>0;right=arcMask&curvature<0;
mode(left)={'ARC_LEFT'};mode(right)={'ARC_RIGHT'};
radius(arcMask)=1./abs(curvature(arcMask));
speedLimit=config.robot.maxLinearSpeed*ones(n,1);
b=config.robot.wheelBase;
for i=find(arcMask).'
    kappa=curvature(i);
    angularLimit=config.robot.maxAngularSpeed/max(abs(kappa),eps);
    wheelFactor=max(abs([1-b*kappa/2,1+b*kappa/2]));
    wheelLimit=config.robot.maxWheelSpeed/max(wheelFactor,eps);
    speedLimit(i)=min([speedLimit(i),angularLimit,wheelLimit]);
end
reference=struct('x',path(:,1),'y',path(:,2),'theta',theta, ...
    'v',zeros(n,1),'omega',zeros(n,1),'mode',{mode},'radius',radius, ...
    'speedLimit',speedLimit,'time',zeros(n,1), ...
    'linearAcceleration',zeros(n,1),'angularAcceleration',zeros(n,1));
reference=generateSpeedProfile(reference,config);
end
