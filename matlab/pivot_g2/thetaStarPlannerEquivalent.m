function result = thetaStarPlannerEquivalent(occupancy,startPoint,goalPoint, ...
        resolution,costPotential,costWeight,captureTrace)
%THETASTARPLANNEREQUIVALENT Lazy-style Theta* co line-of-sight tren grid.
if nargin<5||isempty(costPotential),costPotential=zeros(size(occupancy));end
if nargin<6,costWeight=0;end
if nargin<7,captureTrace=false;end
name='THETA_STAR';occupancy=logical(occupancy);
[rowCount,columnCount]=size(occupancy);
startColumn=floor(startPoint(1)/resolution)+1;startRow=floor(startPoint(2)/resolution)+1;
goalColumn=floor(goalPoint(1)/resolution)+1;goalRow=floor(goalPoint(2)/resolution)+1;
result=struct('name',name,'plugin','nav2_theta_star_planner::ThetaStarPlanner', ...
    'implementation','MATLAB_EQUIVALENT','success',false,'path',zeros(0,2), ...
    'poses',zeros(0,3),'modes',{cell(0,1)},'radii',zeros(0,1), ...
    'trace',struct('currentCells',zeros(0,2),'newCells',{cell(0,1)}, ...
    'planner',name),'planningTime',nan,'expandedNodes',0,'pathCost',inf,'message','');
if ~inside(startRow,startColumn)||~inside(goalRow,goalColumn)|| ...
        occupancy(startRow,startColumn)||occupancy(goalRow,goalColumn)
    result.message='Start/goal khong hop le.';return;
end
nodeCount=rowCount*columnCount;startNode=sub2ind([rowCount columnCount],startRow,startColumn);
goalNode=sub2ind([rowCount columnCount],goalRow,goalColumn);
g=inf(nodeCount,1);g(startNode)=0;parent=zeros(nodeCount,1,'uint32');
parent(startNode)=uint32(startNode);closed=false(nodeCount,1);
heapNodes=zeros(max(64,nodeCount),1,'uint32');heapP=inf(max(64,nodeCount),1);heapSize=0;
push(uint32(startNode),hypot(startRow-goalRow,startColumn-goalColumn));
dirs=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
timer=tic;found=false;trace=result.trace;
while heapSize>0
    current=double(pop());if closed(current),continue;end
    closed(current)=true;result.expandedNodes=result.expandedNodes+1;
    [cr,cc]=ind2sub([rowCount columnCount],current);newly=zeros(0,2);
    if captureTrace,trace.currentCells(end+1,:)=[cr cc];end %#ok<AGROW>
    if current==goalNode,found=true;if captureTrace,trace.newCells{end+1,1}=newly;end;break;end
    for d=1:8
        nr=cr+dirs(d,1);nc=cc+dirs(d,2);
        if ~inside(nr,nc)||occupancy(nr,nc),continue;end
        neighbor=sub2ind([rowCount columnCount],nr,nc);
        p=double(parent(current));[pr,pc]=ind2sub([rowCount columnCount],p);
        if p>0&&hasLineOfSight(pr,pc,nr,nc)
            distance=hypot(nr-pr,nc-pc);
            tentative=g(p)+distance*(1+costWeight*costPotential(nr,nc));newParent=p;
        else
            distance=hypot(nr-cr,nc-cc);
            tentative=g(current)+distance*(1+costWeight*costPotential(nr,nc));newParent=current;
        end
        if tentative+eps<g(neighbor)
            g(neighbor)=tentative;parent(neighbor)=uint32(newParent);
            h=hypot(nr-goalRow,nc-goalColumn);push(uint32(neighbor),tentative+h+1e-6*h);
            if captureTrace,newly(end+1,:)=[nr nc];end %#ok<AGROW>
        end
    end
    if captureTrace,trace.newCells{end+1,1}=unique(newly,'rows');end
end
result.planningTime=toc(timer);result.trace=trace;
if ~found,result.message='Khong tim duoc duong.';return;end
nodes=zeros(nodeCount,1,'uint32');count=1;node=goalNode;nodes(count)=uint32(node);
while node~=startNode,node=double(parent(node));count=count+1;nodes(count)=uint32(node);end
nodes=double(flipud(nodes(1:count)));[pathRows,pathColumns]=ind2sub([rowCount columnCount],nodes);
path=gridToWorld(pathRows,pathColumns,resolution);heading=atan2(diff(path(:,2)),diff(path(:,1)));
result.path=path;result.poses=[path [heading;heading(end)]];
result.modes=repmat({'STRAIGHT'},size(path,1),1);result.radii=inf(size(path,1),1);
result.pathCost=g(goalNode);result.success=true;result.message='OK';

    function tf=inside(row,column),tf=row>=1&&row<=rowCount&&column>=1&&column<=columnCount;end
    function tf=hasLineOfSight(r0,c0,r1,c1)
        losCount=max(abs(r1-r0),abs(c1-c0))+1;
        losRows=round(linspace(r0,r1,losCount));
        losColumns=round(linspace(c0,c1,losCount));
        indices=sub2ind([rowCount columnCount],losRows,losColumns);
        tf=~any(occupancy(indices));
        if tf&&numel(losRows)>1
            for q=2:numel(losRows)
                if losRows(q)~=losRows(q-1)&&losColumns(q)~=losColumns(q-1) && ...
                        (occupancy(losRows(q-1),losColumns(q))|| ...
                         occupancy(losRows(q),losColumns(q-1)))
                    tf=false;return;
                end
            end
        end
    end
    function push(id,pv)
        heapSize=heapSize+1;if heapSize>numel(heapNodes),heapNodes(end+nodeCount,1)=0;heapP(end+nodeCount,1)=inf;end
        idx=heapSize;while idx>1,p=floor(idx/2);if heapP(p)<=pv,break;end;heapNodes(idx)=heapNodes(p);heapP(idx)=heapP(p);idx=p;end
        heapNodes(idx)=id;heapP(idx)=pv;
    end
    function id=pop()
        id=heapNodes(1);last=heapNodes(heapSize);lp=heapP(heapSize);heapSize=heapSize-1;if heapSize==0,return;end
        idx=1;while true,l=2*idx;if l>heapSize,break;end;r=l+1;ch=l;if r<=heapSize&&heapP(r)<heapP(l),ch=r;end;if heapP(ch)>=lp,break;end;heapNodes(idx)=heapNodes(ch);heapP(idx)=heapP(ch);idx=ch;end
        heapNodes(idx)=last;heapP(idx)=lp;
    end
end
