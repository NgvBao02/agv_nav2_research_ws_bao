function candidates = generateTransitionCurveCandidates(corner,radii,map,config)
%GENERATETRANSITIONCURVECANDIDATES Quintic Bezier G2 thay cung tron dot ngot.
% Ba control point dau nam tren tiep tuyen vao, ba control point cuoi nam
% tren tiep tuyen ra, nen curvature bang 0 tai hai diem ghep voi doan thang.
template=struct('radius',nan,'valid',false,'reason','', ...
    'arc',struct([]),'poses',zeros(0,3),'minimumClearance',nan, ...
    'predictedTime',inf,'vArc',nan,'omega',nan, ...
    'leftWheelSpeed',nan,'rightWheelSpeed',nan, ...
    'footprintCheckTime',0,'radiusProfile',zeros(0,1), ...
    'speedLimitProfile',zeros(0,1),'curvatureProfile',zeros(0,1), ...
    'curvatureEnergy',nan,'curveType','QUINTIC_G2_TRANSITION', ...
    'curveOnlyPredictedTime',inf,'comparisonWindowDistance',nan, ...
    'timeProfile',struct());
candidates=repmat(template,numel(radii),1);
for k=1:numel(radii)
    item=template;designRadius=radii(k);item.radius=designRadius;
    angleMagnitude=abs(corner.turnAngle);
    if angleMagnitude<1e-6||angleMagnitude>=pi-1e-6
        item.reason='Goc re khong nam trong (0,pi).';candidates(k)=item;continue;
    end
    tangentDistance=designRadius*tan(angleMagnitude/2);
    maximumTangentDistance=config.maxCornerRadiusFraction* ...
        min(corner.lengthBefore,corner.lengthAfter);
    if tangentDistance>maximumTangentDistance+config.numericTolerance
        item.reason='Doan ke khong du dai cho transition curve.';
        candidates(k)=item;continue;
    end
    p0=corner.vertex-tangentDistance*corner.inDirection;
    p5=corner.vertex+tangentDistance*corner.outDirection;
    fraction=config.adaptiveSelection.bezierControlFraction;
    if ~isfinite(fraction)||fraction<=0||fraction>=0.5
        item.reason='bezierControlFraction phai nam trong (0,0.5).';
        candidates(k)=item;continue;
    end
    controlDistance=max(eps,fraction*tangentDistance);
    controls=[p0; ...
        p0+controlDistance*corner.inDirection; ...
        p0+2*controlDistance*corner.inDirection; ...
        p5-2*controlDistance*corner.outDirection; ...
        p5-controlDistance*corner.outDirection; ...
        p5];
    nominalLength=max(designRadius*angleMagnitude,norm(p5-p0));
    sampleCount=max(7,ceil(nominalLength/config.arcSampleSpacing)+1);
    [points,firstDerivative,secondDerivative,~]= ...
        sampleWithMaximumChord(controls,sampleCount,config.arcSampleSpacing);
    sampleCount=size(points,1);
    derivativeNorm=hypot(firstDerivative(:,1),firstDerivative(:,2));
    if any(~isfinite(points),'all')||any(~isfinite(derivativeNorm))|| ...
            any(derivativeNorm<=1e-10)
        item.reason='Transition curve co dao ham suy bien/khong huu han.';
        candidates(k)=item;continue;
    end
    curvature=(firstDerivative(:,1).*secondDerivative(:,2)- ...
        firstDerivative(:,2).*secondDerivative(:,1))./max(derivativeNorm.^3,eps);
    if max(abs(curvature([1 end])))>1e-8
        item.reason='Control points khong thoa curvature=0 tai diem ghep G2.';
        candidates(k)=item;continue;
    end
    curvature([1 end])=0;
    if any(sign(curvature(abs(curvature)>1e-7))~=sign(corner.turnAngle))
        item.reason='Transition curve tao diem uon nguoc chieu.';
        candidates(k)=item;continue;
    end
    radiusProfile=inf(sampleCount,1);curved=abs(curvature)>1e-8;
    radiusProfile(curved)=1./abs(curvature(curved));
    if any(radiusProfile(curved)<config.robot.wheelBase/2-1e-10)
        item.reason='Do cong cuc dai lam banh trong phai dao chieu.';
        candidates(k)=item;continue;
    end
    speedLimit=config.robot.maxLinearSpeed*ones(sampleCount,1);
    omegaProfile=zeros(sampleCount,1);left=zeros(sampleCount,1);right=zeros(sampleCount,1);
    for i=1:sampleCount
        kappa=curvature(i);
        if abs(kappa)>1e-10
            angularLimit=config.robot.maxAngularSpeed/abs(kappa);
            wheelFactor=max(abs([1-config.robot.wheelBase*kappa/2, ...
                1+config.robot.wheelBase*kappa/2]));
            wheelLimit=config.robot.maxWheelSpeed/max(wheelFactor,eps);
            speedLimit(i)=min([speedLimit(i),angularLimit,wheelLimit]);
        end
        omegaProfile(i)=speedLimit(i)*kappa;
        left(i)=speedLimit(i)-config.robot.wheelBase*omegaProfile(i)/2;
        right(i)=speedLimit(i)+config.robot.wheelBase*omegaProfile(i)/2;
    end
    if any(left<-1e-10)||any(right<-1e-10)
        item.reason='Transition curve yeu cau banh quay nguoc.';
        candidates(k)=item;continue;
    end
    points(1,:)=p0;points(end,:)=p5;
    heading=unwrap(atan2(firstDerivative(:,2),firstDerivative(:,1)));
    poses=[points heading];ds=hypot(diff(points(:,1)),diff(points(:,2)));
    segmentTime=2*ds./max(speedLimit(1:end-1)+speedLimit(2:end),eps);
    predictedTime=sum(segmentTime);
    item.arc=struct('radius',designRadius,'center',[nan nan], ...
        'startPoint',p0,'endPoint',p5,'poses',poses, ...
        'length',sum(ds),'turnDirection',corner.turnDirection, ...
        'tangentDistance',tangentDistance,'curveType',item.curveType);
    item.poses=poses;item.predictedTime=predictedTime;
    item.curveOnlyPredictedTime=predictedTime;
    item.vArc=min(speedLimit);[~,peak]=max(abs(omegaProfile));
    item.omega=omegaProfile(peak);item.leftWheelSpeed=max(abs(left));
    item.rightWheelSpeed=max(abs(right));item.radiusProfile=radiusProfile;
    item.speedLimitProfile=speedLimit;item.curvatureProfile=curvature;
    segmentCurvatureSquared=0.5*(curvature(1:end-1).^2+ ...
        curvature(2:end).^2);
    item.curvatureEnergy=sum(segmentCurvatureSquared.*ds);
    timer=tic;safety=evaluatePoseSequenceSafety(poses,map,config);
    item.footprintCheckTime=toc(timer);
    item.minimumClearance=safety.minimumClearance;
    if ~safety.safe
        item.reason=sprintf('Transition footprint khong an toan (min %.3f m).', ...
            item.minimumClearance);
    else
        item.valid=true;item.reason='Quintic G2 hop le.';
    end
    candidates(k)=item;
end
end

function [point,first,second,t]=sampleWithMaximumChord(control,initialCount,spacing)
% Tang mat do mau den khi moi chord khong dai hon spacing yeu cau.
sampleCount=initialCount;
while true
    t=linspace(0,1,sampleCount).';
    [point,first,second]=quinticGeometry(control,t);
    chord=hypot(diff(point(:,1)),diff(point(:,2)));
    if isempty(chord)||max(chord)<=spacing*(1+1e-6),return;end
    if sampleCount>=2^18,error('Khong hoi tu lay mau transition curve.');end
    sampleCount=2*(sampleCount-1)+1;
end
end

function [point,first,second]=quinticGeometry(control,t)
% Bernstein bac 5 va hai dao ham theo t.
point=zeros(numel(t),2);first=zeros(numel(t),2);second=zeros(numel(t),2);
for i=0:5
    basis=nchoosek(5,i)*(1-t).^(5-i).*t.^i;
    point=point+basis*control(i+1,:);
end
for i=0:4
    basis=nchoosek(4,i)*(1-t).^(4-i).*t.^i;
    first=first+5*basis*(control(i+2,:)-control(i+1,:));
end
for i=0:3
    basis=nchoosek(3,i)*(1-t).^(3-i).*t.^i;
    second=second+20*basis* ...
        (control(i+3,:)-2*control(i+2,:)+control(i+1,:));
end
end
