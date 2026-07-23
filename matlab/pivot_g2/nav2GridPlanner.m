function result = nav2GridPlanner(occupancy,startPoint,goalPoint,resolution, ...
        plannerName,costPotential,costWeight,captureTrace)
%NAV2GRIDPLANNER Tai hien NavFn/Smac2D bang tim kiem grid 8 lien ket.
if nargin<6||isempty(costPotential),costPotential=zeros(size(occupancy));end
if nargin<7,costWeight=0;end
if nargin<8,captureTrace=false;end
plannerName=upper(char(plannerName));
useHeuristic=~strcmp(plannerName,'NAVFN_DIJKSTRA');
occupancy=logical(occupancy);
[rows,columns]=size(occupancy);
startColumn=floor(startPoint(1)/resolution)+1;
startRow=floor(startPoint(2)/resolution)+1;
goalColumn=floor(goalPoint(1)/resolution)+1;
goalRow=floor(goalPoint(2)/resolution)+1;
trace=struct('currentCells',zeros(0,2),'newCells',{cell(0,1)}, ...
    'planner',plannerName);
result=emptyPlannerResult(plannerName);
result.trace=trace;
if ~inside(startRow,startColumn)||~inside(goalRow,goalColumn)|| ...
        occupancy(startRow,startColumn)||occupancy(goalRow,goalColumn)
    result.message='Start/goal khong hop le.';return;
end
nodeCount=rows*columns;
startNode=sub2ind([rows columns],startRow,startColumn);
goalNode=sub2ind([rows columns],goalRow,goalColumn);
gScore=inf(nodeCount,1);gScore(startNode)=0;
parent=zeros(nodeCount,1,'uint32');closed=false(nodeCount,1);
heapNodes=zeros(max(64,nodeCount),1,'uint32');
heapPriority=inf(max(64,nodeCount),1);heapSize=0;
push(uint32(startNode),heuristic(startRow,startColumn));
directions=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
stepCost=[1;1;1;1;sqrt(2);sqrt(2);sqrt(2);sqrt(2)];
timer=tic;goalFound=false;
while heapSize>0
    current=double(pop());
    if closed(current),continue;end
    closed(current)=true;
    result.expandedNodes=result.expandedNodes+1;
    [currentRow,currentColumn]=ind2sub([rows columns],current);
    newly=zeros(0,2);
    if captureTrace,trace.currentCells(end+1,:)=[currentRow currentColumn];end %#ok<AGROW>
    if current==goalNode
        goalFound=true;
        if captureTrace,trace.newCells{end+1,1}=newly;end
        break;
    end
    for d=1:8
        nextRow=currentRow+directions(d,1);
        nextColumn=currentColumn+directions(d,2);
        if ~inside(nextRow,nextColumn)||occupancy(nextRow,nextColumn),continue;end
        if d>4 && (occupancy(currentRow,nextColumn)||occupancy(nextRow,currentColumn))
            continue; % cam cat cheo qua goc vat can
        end
        nextNode=sub2ind([rows columns],nextRow,nextColumn);
        traversalMultiplier=1+costWeight*costPotential(nextRow,nextColumn);
        tentative=gScore(current)+stepCost(d)*traversalMultiplier;
        if tentative+eps<gScore(nextNode)
            gScore(nextNode)=tentative;parent(nextNode)=uint32(current);
            h=heuristic(nextRow,nextColumn);
            push(uint32(nextNode),tentative+h+1e-6*h);
            if captureTrace,newly(end+1,:)=[nextRow nextColumn];end %#ok<AGROW>
        end
    end
    if captureTrace,trace.newCells{end+1,1}=unique(newly,'rows');end
end
result.planningTime=toc(timer);result.trace=trace;
if ~goalFound,result.message='Khong tim duoc duong.';return;end
nodes=zeros(nodeCount,1,'uint32');count=1;node=goalNode;nodes(count)=uint32(node);
while node~=startNode
    node=double(parent(node));
    if node==0,result.message='Loi truy vet.';return;end
    count=count+1;nodes(count)=uint32(node);
end
nodes=double(flipud(nodes(1:count)));
[pathRows,pathColumns]=ind2sub([rows columns],nodes);
path=gridToWorld(pathRows,pathColumns,resolution);
result=finishPlannerResult(result,path,gScore(goalNode));

    function tf=inside(row,column)
        tf=row>=1&&row<=rows&&column>=1&&column<=columns;
    end
    function h=heuristic(row,column)
        if useHeuristic,h=hypot(row-goalRow,column-goalColumn);else,h=0;end
    end
    function push(nodeId,priority)
        heapSize=heapSize+1;
        if heapSize>numel(heapNodes)
            heapNodes(end+nodeCount,1)=uint32(0);heapPriority(end+nodeCount,1)=inf;
        end
        index=heapSize;
        while index>1
            p=floor(index/2);if heapPriority(p)<=priority,break;end
            heapNodes(index)=heapNodes(p);heapPriority(index)=heapPriority(p);index=p;
        end
        heapNodes(index)=nodeId;heapPriority(index)=priority;
    end
    function nodeId=pop()
        nodeId=heapNodes(1);lastNode=heapNodes(heapSize);lastP=heapPriority(heapSize);
        heapSize=heapSize-1;if heapSize==0,return;end
        index=1;
        while true
            left=2*index;if left>heapSize,break;end
            right=left+1;child=left;
            if right<=heapSize&&heapPriority(right)<heapPriority(left),child=right;end
            if heapPriority(child)>=lastP,break;end
            heapNodes(index)=heapNodes(child);heapPriority(index)=heapPriority(child);index=child;
        end
        heapNodes(index)=lastNode;heapPriority(index)=lastP;
    end
end

function result=emptyPlannerResult(name)
result=struct('name',name,'plugin','','implementation','MATLAB_EQUIVALENT', ...
    'success',false,'path',zeros(0,2),'poses',zeros(0,3), ...
    'modes',{cell(0,1)},'radii',zeros(0,1),'trace',struct(), ...
    'planningTime',nan,'expandedNodes',0,'pathCost',inf,'message','');
end

function result=finishPlannerResult(result,path,pathCost)
result.path=path;
if size(path,1)>1
    segmentHeading=atan2(diff(path(:,2)),diff(path(:,1)));
    theta=[segmentHeading;segmentHeading(end)];
else,theta=0;end
result.poses=[path theta];
result.modes=repmat({'STRAIGHT'},size(path,1),1);
result.radii=inf(size(path,1),1);
result.pathCost=pathCost;result.success=true;result.message='OK';
end
