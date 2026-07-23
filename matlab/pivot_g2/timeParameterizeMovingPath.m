function profile = timeParameterizeMovingPath(points,curvature,speedLimit, ...
        robot,startSpeed,endSpeed)
%TIMEPARAMETERIZEMOVINGPATH Profile chung cho moi phuong an chuyen dong.
% Gioi han v, omega, toc do hai banh, gia toc tuyen tinh va gia toc goc.
template=struct('valid',false,'reason','','linearSpeed',zeros(0,1), ...
    'angularSpeed',zeros(0,1),'time',zeros(0,1),'totalTime',inf, ...
    'maxAbsAngularAcceleration',inf,'iterations',0);
profile=template;
points=double(points);curvature=curvature(:);speedLimit=speedLimit(:);
n=size(points,1);
if n<2||size(points,2)~=2||numel(curvature)~=n||numel(speedLimit)~=n
    profile.reason='Path time parameterization co kich thuoc khong hop le.';return;
end
if any(~isfinite(points),'all')||any(~isfinite(curvature))|| ...
        any(~isfinite(speedLimit))||any(speedLimit<=0)
    profile.reason='Path co mau hoac speed limit khong hop le.';return;
end
ds=hypot(diff(points(:,1)),diff(points(:,2)));
if any(~isfinite(ds))||any(ds<=1e-12)
    profile.reason='Path chuyen dong co diem trung hoac doan khong hop le.';return;
end

absoluteCurvature=abs(curvature);
caps=min(speedLimit,robot.maxLinearSpeed);
curved=absoluteCurvature>1e-12;
caps(curved)=min(caps(curved), ...
    robot.maxAngularSpeed./absoluteCurvature(curved));
wheelFactor=max(abs([1-robot.wheelBase*curvature/2, ...
    1+robot.wheelBase*curvature/2]),[],2);
caps=min(caps,robot.maxWheelSpeed./max(wheelFactor,eps));
caps(1)=min(caps(1),max(0,startSpeed));
caps(end)=min(caps(end),max(0,endSpeed));
interior=caps(2:end-1);
if any(caps<0)||any(~isfinite(caps))||any(interior<=0)
    profile.reason='Khong ton tai speed cap duong tren path.';return;
end

converged=false;v=caps;
for iteration=1:40
    v=enforceLinearLimits(caps,ds,robot);
    changed=false;
    for i=2:n
        dt=segmentTime(ds(i-1),v(i-1),v(i));
        if ~isfinite(dt)
            profile.reason='Doan co quang duong duong nhung toc do bang 0.';return;
        end
        omegaBefore=v(i-1)*curvature(i-1);
        omegaAfter=v(i)*curvature(i);
        angularAcceleration=abs(omegaAfter-omegaBefore)/dt;
        if angularAcceleration>robot.maxAngularAcceleration*(1+1e-6)
            factor=max(0.1,0.995*sqrt( ...
                robot.maxAngularAcceleration/angularAcceleration));
            caps(i-1)=caps(i-1)*factor;caps(i)=caps(i)*factor;
            changed=true;
        end
    end
    profile.iterations=iteration;
    if ~changed,converged=true;break;end
end
if ~converged
    profile.reason='Lap gioi han gia toc goc khong hoi tu.';return;
end
v=enforceLinearLimits(caps,ds,robot);
omega=v.*curvature;time=zeros(n,1);maximumAngularAcceleration=0;
for i=2:n
    dt=segmentTime(ds(i-1),v(i-1),v(i));
    if ~isfinite(dt),profile.reason='Time profile khong kha thi.';return;end
    time(i)=time(i-1)+dt;
    maximumAngularAcceleration=max(maximumAngularAcceleration, ...
        abs(omega(i)-omega(i-1))/dt);
end
if maximumAngularAcceleration>robot.maxAngularAcceleration*(1+1e-4)
    profile.reason='Time profile van vuot gioi han gia toc goc.';return;
end
profile.valid=true;profile.reason='Hop le.';profile.linearSpeed=v;
profile.angularSpeed=omega;profile.time=time;profile.totalTime=time(end);
profile.maxAbsAngularAcceleration=maximumAngularAcceleration;
end

function speed=enforceLinearLimits(caps,ds,robot)
speed=caps;
for pass=1:3
    for i=2:numel(speed)
        reachable=sqrt(max(0,speed(i-1)^2+ ...
            2*robot.maxLinearAcceleration*ds(i-1)));
        speed(i)=min(speed(i),reachable);
    end
    for i=numel(speed)-1:-1:1
        reachable=sqrt(max(0,speed(i+1)^2+ ...
            2*robot.maxLinearDeceleration*ds(i)));
        speed(i)=min(speed(i),reachable);
    end
end
end

function duration=segmentTime(lengthValue,startSpeed,endSpeed)
if lengthValue<=1e-12,duration=0;return;end
speedSum=startSpeed+endSpeed;
if speedSum<=1e-12,duration=inf;else,duration=2*lengthValue/speedSum;end
end
