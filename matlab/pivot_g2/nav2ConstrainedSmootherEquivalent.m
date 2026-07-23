function [smoothed,info] = nav2ConstrainedSmootherEquivalent( ...
        path,planningOccupancy,map,config,comparison)
%NAV2CONSTRAINEDSMOOTHEREQUIVALENT Toi uu rang buoc khong can Ceres.
% Day la MATLAB-equivalent theo cac thanh phan objective duoc Nav2 cong bo,
% khong phai ban dich bit-for-bit cua nav2_constrained_smoother C++.
p=comparison.constrained;original=path;smoothed=path;
potential=computeCostmapPotential(planningOccupancy,map.resolution, ...
    max(p.targetClearance,map.resolution));
[potentialDx,potentialDy]=gradient(potential,map.resolution);
info=struct('plugin','nav2_constrained_smoother::ConstrainedSmoother', ...
    'implementation','MATLAB_EQUIVALENT_OBJECTIVE', ...
    'iterations',0,'converged',false,'costmapRejectedUpdates',0, ...
    'footprintRollback',false,'maximumDisplacement',0, ...
    'coreTime',0,'validation',struct(), ...
    'objectiveStart',pathObjective(path,original,p), ...
    'objectiveEnd',nan,'message','');
if size(path,1)<=3
    info.message='Segment qua ngan de toi uu.';
    info.validation=validatePostprocessedPath(smoothed,map,config, ...
        comparison.reference.sampleSpacing);info.objectiveEnd=info.objectiveStart;return;
end
coreTimer=tic;
for iteration=1:p.maxIterations
    previous=smoothed;proposal=smoothed;
    for i=2:size(smoothed,1)-1
        laplacian=0.5*(smoothed(i-1,:)+smoothed(i+1,:))-smoothed(i,:);
        dataForce=original(i,:)-smoothed(i,:);
        [gradientX,gradientY]=samplePotentialGradient(smoothed(i,:), ...
            potentialDx,potentialDy,map);
        obstacleForce=-[gradientX gradientY];
        curvature=localCurvature(smoothed,i);
        excess=max(0,abs(curvature)-1/max(p.minimumTurningRadius,eps));
        curvatureForce=laplacian*excess*max(p.minimumTurningRadius,eps);
        update=p.stepSize*(p.dataWeight*dataForce + ...
            p.smoothWeight*laplacian + p.obstacleWeight*obstacleForce + ...
            p.curvatureWeight*curvatureForce);
        candidate=smoothed(i,:)+update;
        displacement=candidate-original(i,:);
        if norm(displacement)>p.maximumDisplacement
            candidate=original(i,:)+p.maximumDisplacement* ...
                displacement/norm(displacement);
        end
        if pointAdmissible(candidate,planningOccupancy,map)
            proposal(i,:)=candidate;
        else
            info.costmapRejectedUpdates=info.costmapRejectedUpdates+1;
        end
    end
    smoothed=proposal;info.iterations=iteration;
    maximumChange=max(hypot(smoothed(:,1)-previous(:,1), ...
        smoothed(:,2)-previous(:,2)));
    if maximumChange<p.tolerance,info.converged=true;break;end
end
info.coreTime=toc(coreTimer);
validation=validatePostprocessedPath(smoothed,map,config, ...
    comparison.reference.sampleSpacing);
if ~validation.safe
    smoothed=original;info.footprintRollback=true;
    validation=validatePostprocessedPath(smoothed,map,config, ...
        comparison.reference.sampleSpacing);
end
info.validation=validation;
info.maximumDisplacement=max(hypot(smoothed(:,1)-original(:,1), ...
    smoothed(:,2)-original(:,2)));
info.objectiveEnd=pathObjective(smoothed,original,p);
if info.footprintRollback
    info.message='Fallback ve path goc do footprint/clearance.';
elseif info.converged,info.message='Hoi tu.';else,info.message='Dat gioi han lap.';end
end

function [gx,gy]=samplePotentialGradient(point,fieldX,fieldY,map)
column=point(1)/map.resolution+0.5;row=point(2)/map.resolution+0.5;
gx=interp2(fieldX,column,row,'linear',0);gy=interp2(fieldY,column,row,'linear',0);
end
function tf=pointAdmissible(point,occupancy,map)
[row,column,valid]=worldToGrid(point,map);tf=valid&&~occupancy(row,column);
end
function value=localCurvature(path,i)
a=path(i,:)-path(i-1,:);b=path(i+1,:)-path(i,:);
la=norm(a);lb=norm(b);
if la<eps||lb<eps,value=0;return;end
angle=atan2(a(1)*b(2)-a(2)*b(1),dot(a,b));
value=angle/max(0.5*(la+lb),eps);
end
function value=pathObjective(path,original,p)
data=sum((path-original).^2,'all');
if size(path,1)>2
    second=path(3:end,:)-2*path(2:end-1,:)+path(1:end-2,:);
    smooth=sum(second.^2,'all');
else,smooth=0;end
value=p.dataWeight*data+p.smoothWeight*smooth;
end
