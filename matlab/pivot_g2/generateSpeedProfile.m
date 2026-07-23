function reference = generateSpeedProfile(reference, config)
%GENERATESPEEDPROFILE Tao profile gioi han v, omega, gia toc va toc do banh.
n = numel(reference.x);
if n < 2
    error('Reference phai co it nhat hai mau.');
end
robot = config.robot;
ds = hypot(diff(reference.x),diff(reference.y));
dtheta = abs(diff(reference.theta));
isPivot = startsWith(reference.mode,'PIVOT');

targetV = min(reference.speedLimit,robot.maxLinearSpeed);
targetV(isPivot) = 0;
targetV(1) = 0; targetV(end) = 0;
v = targetV;
v(1) = 0;
for i = 2:n
    reachable = sqrt(max(0,v(i-1)^2 + 2*robot.maxLinearAcceleration*ds(i-1)));
    v(i) = min(v(i),reachable);
end
v(end) = 0;
for i = n-1:-1:1
    reachable = sqrt(max(0,v(i+1)^2 + 2*robot.maxLinearDeceleration*ds(i)));
    v(i) = min(v(i),reachable);
end

desiredOmega = zeros(n,1);
arcMask = startsWith(reference.mode,'ARC') & isfinite(reference.radius) & ...
    reference.radius>0;
for i = 1:n
    if arcMask(i)
        turnSign = 1;
        if strcmp(reference.mode{i},'ARC_RIGHT'), turnSign = -1; end
        desiredOmega(i) = turnSign*v(i)/reference.radius(i);
    end
end

% Profile goc tam giac/hinh thang tren tung cum pivot.
pivotIndices = find(isPivot);
if ~isempty(pivotIndices)
    runStarts = pivotIndices([true;diff(pivotIndices)>1]);
    runEnds = pivotIndices([diff(pivotIndices)>1;true]);
    for q = 1:numel(runStarts)
        indices = (runStarts(q):runEnds(q)).';
        signTurn = sign(reference.theta(indices(end))-reference.theta(indices(1)));
        magnitude = robot.maxAngularSpeed*ones(numel(indices),1);
        magnitude(1) = 0;
        for j = 2:numel(indices)
            da = abs(reference.theta(indices(j))-reference.theta(indices(j-1)));
            magnitude(j) = min(magnitude(j),sqrt(magnitude(j-1)^2 + ...
                2*robot.maxAngularAcceleration*da));
        end
        magnitude(end) = 0;
        for j = numel(indices)-1:-1:1
            da = abs(reference.theta(indices(j+1))-reference.theta(indices(j)));
            magnitude(j) = min(magnitude(j),sqrt(magnitude(j+1)^2 + ...
                2*robot.maxAngularAcceleration*da));
        end
        desiredOmega(indices) = signTurn*magnitude;
    end
end
desiredOmega = max(-robot.maxAngularSpeed,min(robot.maxAngularSpeed,desiredOmega));

time = calculateTime(v,desiredOmega,ds,dtheta,config.dt,robot);
omega = desiredOmega;
% Moi cung phai tang/giam omega ngay trong chinh cum ARC. Khong cho profile
% goc ro sang mau STRAIGHT, vi khi do reference bao di thang nhung robot van
% nhan lenh quay va se vuot goc. Pivot da co profile rieng o tren.
for pass = 1:4
    omega = desiredOmega;
    omega(~arcMask & ~isPivot) = 0;
    omega = limitArcAngularProfile(omega,arcMask,time, ...
        robot.maxAngularAcceleration);
    % Giu tinh nhat quan dong hoc tren cung: neu omega dang tang/giam theo
    % gioi han gia toc goc thi v cung phai thoa v=|omega|R. Sau do lan
    % truyen xuoi/nguoc lai dam bao gioi han gia toc tuyen tinh.
    v(arcMask)=min(v(arcMask),abs(omega(arcMask)).*reference.radius(arcMask));
    v(isPivot)=0;
    for i=2:n
        reachable=sqrt(max(0,v(i-1)^2+2*robot.maxLinearAcceleration*ds(i-1)));
        v(i)=min(v(i),reachable);
    end
    for i=n-1:-1:1
        reachable=sqrt(max(0,v(i+1)^2+2*robot.maxLinearDeceleration*ds(i)));
        v(i)=min(v(i),reachable);
    end
    omega(arcMask)=sign(omega(arcMask)).*min(abs(omega(arcMask)), ...
        v(arcMask)./reference.radius(arcMask));
    % Gioi han dong hoc hai banh tai moi mau.
    left = v-robot.wheelBase*omega/2;
    right = v+robot.wheelBase*omega/2;
    scale = max([ones(n,1),abs(left)/robot.maxWheelSpeed, ...
        abs(right)/robot.maxWheelSpeed],[],2);
    v = v./scale;
    omega = omega./scale;
    time = calculateTime(v,omega,ds,dtheta,config.dt,robot);
end

dtVector = max(diff(time),eps);
linearAcceleration = [0;diff(v)./dtVector];
angularAcceleration = [0;diff(omega)./dtVector];
reference.v = v;
reference.omega = omega;
reference.time = time;
reference.linearAcceleration = linearAcceleration;
reference.angularAcceleration = angularAcceleration;
reference.leftWheelVelocity = v-robot.wheelBase*omega/2;
reference.rightWheelVelocity = v+robot.wheelBase*omega/2;
end

function time = calculateTime(v,omega,ds,dtheta,minimumDt,robot)
n = numel(v);
segmentTime = zeros(n-1,1);
for i = 1:n-1
    if ds(i) > 1e-12
        speedSum = v(i)+v(i+1);
        if speedSum > 1e-8
            segmentTime(i) = 2*ds(i)/speedSum;
        else
            segmentTime(i) = 2*sqrt(ds(i)/robot.maxLinearAcceleration);
        end
    elseif dtheta(i) > 1e-12
        omegaSum = abs(omega(i))+abs(omega(i+1));
        if omegaSum > 1e-8
            segmentTime(i) = 2*dtheta(i)/omegaSum;
        else
            segmentTime(i) = 2*sqrt(dtheta(i)/robot.maxAngularAcceleration);
        end
    else
        segmentTime(i) = minimumDt;
    end
    segmentTime(i) = max(segmentTime(i),minimumDt);
end
time = [0;cumsum(segmentTime)];
end

function omega = limitArcAngularProfile(omega,arcMask,time,maxAcceleration)
%LIMITARCANGULARPROFILE Tao ramp omega hai dau tung cum ARC rieng biet.
arcIndices = find(arcMask);
if isempty(arcIndices), return; end
arcSign=sign(omega(arcIndices));
% Tach ca tai diem doi chieu cong. Neu gop ARC_LEFT va ARC_RIGHT thanh mot
% cum, dau cua omega co the bi tong tri tieu va lam hong profile.
newRun=[true;diff(arcIndices)>1|arcSign(2:end)~=arcSign(1:end-1)];
runStarts=arcIndices(newRun);
runEnds=arcIndices([newRun(2:end);true]);
n = numel(omega);
for q = 1:numel(runStarts)
    first = runStarts(q);
    last = runEnds(q);
    turnSign = sign(sum(omega(first:last)));
    if turnSign == 0, continue; end
    magnitude = abs(omega(first:last));

    previousMagnitude = 0;
    for i = first:last
        if i == 1
            deltaTime = 0;
        else
            deltaTime = max(0,time(i)-time(i-1));
        end
        localIndex = i-first+1;
        magnitude(localIndex) = min(magnitude(localIndex), ...
            previousMagnitude+maxAcceleration*deltaTime);
        previousMagnitude = magnitude(localIndex);
    end

    nextMagnitude = 0;
    for i = last:-1:first
        if i == n
            deltaTime = 0;
        else
            deltaTime = max(0,time(i+1)-time(i));
        end
        localIndex = i-first+1;
        magnitude(localIndex) = min(magnitude(localIndex), ...
            nextMagnitude+maxAcceleration*deltaTime);
        nextMagnitude = magnitude(localIndex);
    end
    omega(first:last) = turnSign*magnitude;
end
end
