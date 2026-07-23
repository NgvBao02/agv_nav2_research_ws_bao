function [reference,info] = smoothReferenceTrajectoryCommon(reference,map,config,comparison)
%SMOOTHREFERENCETRAJECTORYCOMMON Lam muot nhe cung mot cach cho moi planner.
% Endpoint, cum pivot va hai mau ke pivot duoc khoa. Mot thay doi chi duoc
% chap nhan neu footprint va clearance van an toan tren ban do vat can goc.

settings=comparison.commonSmoother;
info=struct('enabled',logical(settings.enabled),'applied',false, ...
    'iterations',0,'maximumDisplacement',0,'acceptedBlend',0, ...
    'clearanceValidated',false,'preservesInputG2',true);
if ~settings.enabled || numel(reference.x)<3
    return;
end

original=[reference.x(:) reference.y(:)];
candidate=original;n=size(original,1);
pivotMask=startsWith(reference.mode(:),'PIVOT');
fixed=pivotMask;fixed([1 n])=true;
fixed(2:end)=fixed(2:end)|pivotMask(1:end-1);
fixed(1:end-1)=fixed(1:end-1)|pivotMask(2:end);

for iteration=1:settings.iterations
    previous=candidate;
    for i=2:n-1
        if fixed(i),continue;end
        if norm(candidate(i,:)-candidate(i-1,:))<1e-10 || ...
                norm(candidate(i+1,:)-candidate(i,:))<1e-10
            continue;
        end
        laplacian=0.5*(candidate(i-1,:)+candidate(i+1,:))-candidate(i,:);
        dataCorrection=original(i,:)-candidate(i,:);
        candidate(i,:)=candidate(i,:)+settings.relaxation*( ...
            settings.smoothWeight*laplacian+settings.dataWeight*dataCorrection);
        displacement=candidate(i,:)-original(i,:);
        distance=norm(displacement);
        if distance>settings.maximumDisplacement
            candidate(i,:)=original(i,:)+ ...
                settings.maximumDisplacement*displacement/distance;
        end
    end
    info.iterations=iteration;
    if max(hypot(candidate(:,1)-previous(:,1),candidate(:,2)-previous(:,2)))<1e-6
        break;
    end
end

blend=1;accepted=false;
while blend>=1/64
    trial=original+blend*(candidate-original);
    trialTheta=recomputeMovingHeadings(trial,reference.theta(:),pivotMask);
    if trajectoryIsSafe(trial,trialTheta,pivotMask,map,config,settings.validationStride)
        candidate=trial;accepted=true;break;
    end
    blend=blend/2;
end
if ~accepted
    candidate=original;blend=0;
end

reference.x=candidate(:,1);reference.y=candidate(:,2);
reference.theta=recomputeMovingHeadings(candidate,reference.theta(:),pivotMask);
[reference.mode,reference.radius,reference.speedLimit]=refreshGeometry(reference,config);
reference=generateSpeedProfile(reference,config);
info.maximumDisplacement=max(hypot(candidate(:,1)-original(:,1), ...
    candidate(:,2)-original(:,2)));
info.applied=info.maximumDisplacement>1e-8;
info.preservesInputG2=~info.applied;
info.acceptedBlend=blend;
info.clearanceValidated=accepted;
end

function theta=recomputeMovingHeadings(points,theta,pivotMask)
moving=~pivotMask;
runStarts=find(moving&[true;~moving(1:end-1)]);
runEnds=find(moving&[~moving(2:end);true]);
for q=1:numel(runStarts)
    indices=(runStarts(q):runEnds(q)).';
    if numel(indices)<2,continue;end
    localTheta=zeros(numel(indices),1);
    for k=1:numel(indices)
        if k==1
            direction=points(indices(2),:)-points(indices(1),:);
        elseif k==numel(indices)
            direction=points(indices(end),:)-points(indices(end-1),:);
        else
            direction=points(indices(k+1),:)-points(indices(k-1),:);
        end
        if norm(direction)<1e-12
            localTheta(k)=theta(indices(k));
        else
            localTheta(k)=atan2(direction(2),direction(1));
        end
    end
    localTheta=unwrap(localTheta);
    localTheta=localTheta+2*pi*round((theta(indices(1))-localTheta(1))/(2*pi));
    theta(indices)=localTheta;
end
theta=unwrap(theta);
end

function safe=trajectoryIsSafe(points,theta,~,map,config,stride)
% Khong bo qua pose trong phep chap nhan refinement. Giu tham so stride de
% tuong thich config cu, nhung gia tri >1 khong con lam giam muc an toan.
if stride~=1
    warning('PivotG2:ValidationStrideIgnored', ...
        'validationStride=%d bi bo qua; safety validation luon day du.',stride);
end
validation=evaluatePoseSequenceSafety([points theta],map,config);
safe=validation.safe;
end

function [mode,radius,speedLimit]=refreshGeometry(reference,config)
mode=reference.mode(:);radius=reference.radius(:);
n=numel(reference.x);speedLimit=config.robot.maxLinearSpeed*ones(n,1);
pivotMask=startsWith(mode,'PIVOT');speedLimit(pivotMask)=0;radius(pivotMask)=0;
arcIndices=find(startsWith(mode,'ARC'));
for k=1:numel(arcIndices)
    i=arcIndices(k);before=max(1,i-1);after=min(n,i+1);
    distance=hypot(reference.x(after)-reference.x(before), ...
        reference.y(after)-reference.y(before));
    curvature=0;
    if distance>1e-10
        curvature=wrapAngle(reference.theta(after)-reference.theta(before))/distance;
    end
    if abs(curvature)>1e-4
        radius(i)=1/abs(curvature);
        if curvature>0,mode{i}='ARC_LEFT';else,mode{i}='ARC_RIGHT';end
    elseif ~isfinite(radius(i)) || radius(i)<=0
        mode{i}='STRAIGHT';radius(i)=inf;
    end
    if startsWith(mode{i},'ARC')
        r=max(radius(i),1e-6);
        speedLimit(i)=min([config.robot.maxLinearSpeed, ...
            config.robot.maxAngularSpeed*r, ...
            config.robot.maxWheelSpeed/(1+config.robot.wheelBase/(2*r))]);
    end
end
end
