function [reference,decisions] = plannerResultToReference(plannerResult,map,config,cornerMethod)
%PLANNERRESULTTOREFERENCE Doi path cua planner sang reference chung.
if nargin<4,cornerMethod='PIVOT_ONLY';end
name=plannerResult.name;
if any(strcmpi(name,{'NAVFN_DIJKSTRA','NAVFN_ASTAR','SMAC_2D','THETA_STAR'}))
    reduced=removeCollinearPoints(plannerResult.path);
    corners=detectCorners(reduced);
    [decisions,~]=chooseManeuversForMethod(corners,map,config,cornerMethod);
    reference=buildReferenceTrajectory(reduced,decisions,config);
    reference=generateSpeedProfile(reference,config);
    return;
end
decisions=struct([]);
poses=plannerResult.poses;
if size(poses,1)<2,error('%s khong co du pose.',plannerResult.name);end
keep=true(size(poses,1),1);
for i=2:size(poses,1)
    keep(i)=norm(poses(i,1:2)-poses(i-1,1:2))>1e-10|| ...
        abs(wrapAngle(poses(i,3)-poses(i-1,3)))>1e-10;
end
poses=poses(keep,:);modes=plannerResult.modes(keep);radii=plannerResult.radii(keep);
speedLimit=zeros(size(poses,1),1);
for i=1:size(poses,1)
    if startsWith(modes{i},'PIVOT')
        speedLimit(i)=0;
    elseif startsWith(modes{i},'ARC')&&isfinite(radii(i))&&radii(i)>0
        [~,details]=estimateArcTime(radii(i),ternary(strcmp(modes{i},'ARC_LEFT'),pi/2,-pi/2), ...
            config.robot.maxLinearSpeed,config.robot.maxLinearSpeed,config.robot);
        if details.valid,speedLimit(i)=details.vArc;else,speedLimit(i)=0;end
    else
        speedLimit(i)=config.robot.maxLinearSpeed;
    end
end
reference=struct('x',poses(:,1),'y',poses(:,2),'theta',unwrap(poses(:,3)), ...
    'v',zeros(size(poses,1),1),'omega',zeros(size(poses,1),1), ...
    'mode',{modes(:)},'radius',radii(:),'speedLimit',speedLimit, ...
    'time',zeros(size(poses,1),1),'linearAcceleration',zeros(size(poses,1),1), ...
    'angularAcceleration',zeros(size(poses,1),1));
reference=generateSpeedProfile(reference,config);
end

function value=ternary(condition,a,b)
if condition,value=a;else,value=b;end
end
