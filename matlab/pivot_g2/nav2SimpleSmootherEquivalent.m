function [smoothed,info] = nav2SimpleSmootherEquivalent( ...
        path,planningOccupancy,map,config,comparison)
%NAV2SIMPLESMOOTHEREQUIVALENT MATLAB-equivalent cua SimpleSmoother Nav2.
% Cong thuc, trong so mac dinh, refinement va dieu kien so diem bam theo
% simple_smoother.cpp. Costmap guard dung grid da inflate cua benchmark.
p=comparison.simple;originalInput=path;smoothed=path;
info=baseInfo('nav2_smoother::SimpleSmoother','MATLAB_EQUIVALENT');
if size(path,1)<=4
    info.message='Nav2 bo qua segment co khong qua 4 diem.';
    info.validation=validatePostprocessedPath(smoothed,map,config, ...
        comparison.reference.sampleSpacing);return;
end
coreTimer=tic;
passes=1;
if p.doRefinement,passes=passes+p.refinementCount;end
for pass=1:passes
    source=smoothed;current=smoothed;lastSafe=smoothed;
    change=p.tolerance;iteration=0;
    while change>=p.tolerance
        iteration=iteration+1;change=0;
        if iteration>=p.maxIterations
            current=lastSafe;info.hitIterationLimit=true;break;
        end
        infeasible=false;
        for i=2:size(source,1)-1
            for dimension=1:2
                old=current(i,dimension);
                current(i,dimension)=old + p.dataWeight* ...
                    (source(i,dimension)-old) + p.smoothWeight* ...
                    (current(i+1,dimension)+current(i-1,dimension)-2*old);
                change=change+abs(current(i,dimension)-old);
            end
            if ~pointAdmissible(current(i,:),planningOccupancy,map)
                infeasible=true;break;
            end
        end
        if infeasible
            current=lastSafe;info.costmapRollback=true;break;
        end
        lastSafe=current;
    end
    smoothed=current;info.iterations=info.iterations+iteration;
end
info.coreTime=toc(coreTimer);
info.refinementPasses=passes;
validation=validatePostprocessedPath(smoothed,map,config, ...
    comparison.reference.sampleSpacing);
if ~validation.safe
    smoothed=originalInput;info.footprintRollback=true;
    validation=validatePostprocessedPath(smoothed,map,config, ...
        comparison.reference.sampleSpacing);
end
info.validation=validation;
info.maximumDisplacement=max(hypot(smoothed(:,1)-originalInput(:,1), ...
    smoothed(:,2)-originalInput(:,2)));
info.message=appendMessage(info.message,rollbackMessage(info));
end

function tf=pointAdmissible(point,occupancy,map)
[row,column,valid]=worldToGrid(point,map);
tf=valid&&~occupancy(row,column);
end
function info=baseInfo(plugin,implementation)
info=struct('plugin',plugin,'implementation',implementation,'iterations',0, ...
    'refinementPasses',0,'hitIterationLimit',false, ...
    'costmapRollback',false,'footprintRollback',false, ...
    'maximumDisplacement',0,'coreTime',0,'validation',struct(),'message','');
end
function text=rollbackMessage(info)
if info.footprintRollback,text='Fallback ve path goc do footprint/clearance.';
elseif info.costmapRollback,text='Dung tai path an toan cuoi do costmap.';
elseif info.hitIterationLimit,text='Dat gioi han lap; tra path an toan cuoi.';
else,text='OK';end
end
function out=appendMessage(first,second)
if isempty(first),out=second;else,out=[first ' ' second];end
end
