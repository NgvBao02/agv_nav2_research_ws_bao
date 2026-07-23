function [pathMeters,info] = astar4DirectionTurnPenalty(occupancy,startPoint, ...
        goalPoint,resolution,turnPenalty,forbidReverse,captureTrace)
%ASTAR4DIRECTIONTURNPENALTY A* cai tien co trang thai huong va phat re.
% G(next)=G(current)+1+lambda neu doi huong, voi lambda=K2-1.
% Nhu vay chi phi tong cua buoc re la K2; huong 0 chi dung tai start.
if nargin < 6, forbidReverse=true; end
if nargin < 7, captureTrace=false; end
validateattributes(occupancy,{'numeric','logical'},{'2d','nonempty'});
validateattributes(startPoint,{'numeric'},{'vector','numel',2,'finite'});
validateattributes(goalPoint,{'numeric'},{'vector','numel',2,'finite'});
validateattributes(resolution,{'numeric'},{'scalar','positive','finite'});
validateattributes(turnPenalty,{'numeric'},{'scalar','nonnegative','finite'});
occupancy=logical(occupancy);
[rows,columns]=size(occupancy);
startColumn=floor(startPoint(1)/resolution)+1;
startRow=floor(startPoint(2)/resolution)+1;
goalColumn=floor(goalPoint(1)/resolution)+1;
goalRow=floor(goalPoint(2)/resolution)+1;
trace=struct('currentCells',zeros(0,2),'newCells',{cell(0,1)}, ...
    'planner','IMPROVED_TURN_PENALTY');
info=struct('success',false,'expandedNodes',0,'pathCostCells',inf, ...
    'message','','startCell',[startRow startColumn], ...
    'goalCell',[goalRow goalColumn],'trace',trace, ...
    'planner','IMPROVED_TURN_PENALTY','turnPenalty',turnPenalty, ...
    'turnCostRatio',1+turnPenalty,'numberOfTurns',nan);
pathMeters=zeros(0,2);
if startRow<1 || startRow>rows || startColumn<1 || startColumn>columns
    info.message='Start nam ngoai ban do.'; return;
end
if goalRow<1 || goalRow>rows || goalColumn<1 || goalColumn>columns
    info.message='Goal nam ngoai ban do.'; return;
end
if occupancy(startRow,startColumn) || occupancy(goalRow,goalColumn)
    info.message='Start hoac goal nam trong vat can.'; return;
end

cellCount=rows*columns;
stateCount=5*cellCount; % direction = 0,1,2,3,4
startCell=sub2ind([rows columns],startRow,startColumn);
goalCell=sub2ind([rows columns],goalRow,goalColumn);
startState=stateIndex(startCell,0);
gScore=inf(stateCount,1);
parent=zeros(stateCount,1,'uint32');
closed=false(stateCount,1);
gScore(startState)=0;
heapStates=zeros(max(64,stateCount),1,'uint32');
heapPriority=inf(max(64,stateCount),1);
heapSize=0;
startH=abs(startRow-goalRow)+abs(startColumn-goalColumn);
pushHeap(uint32(startState),startH);
% right, up, left, down trong he row tang theo y.
directions=[0 1;1 0;0 -1;-1 0];
opposite=[3 4 1 2];
goalState=0;
while heapSize>0
    currentState=double(popHeap());
    if closed(currentState),continue;end
    closed(currentState)=true;
    [currentCell,currentDirection]=decodeState(currentState);
    [currentRow,currentColumn]=ind2sub([rows columns],currentCell);
    info.expandedNodes=info.expandedNodes+1;
    newlyDiscovered=zeros(0,2);
    if captureTrace
        trace.currentCells(end+1,:)=[currentRow currentColumn];
    end
    if currentCell==goalCell
        goalState=currentState;
        if captureTrace,trace.newCells{end+1,1}=newlyDiscovered;end
        break;
    end
    for nextDirection=1:4
        if forbidReverse && currentDirection~=0 && ...
                nextDirection==opposite(currentDirection)
            continue;
        end
        nextRow=currentRow+directions(nextDirection,1);
        nextColumn=currentColumn+directions(nextDirection,2);
        if nextRow<1 || nextRow>rows || nextColumn<1 || nextColumn>columns || ...
                occupancy(nextRow,nextColumn)
            continue;
        end
        nextCell=sub2ind([rows columns],nextRow,nextColumn);
        nextState=stateIndex(nextCell,nextDirection);
        directionCost=0;
        if currentDirection~=0 && currentDirection~=nextDirection
            directionCost=turnPenalty;
        end
        tentativeG=gScore(currentState)+1+directionCost;
        if tentativeG+eps<gScore(nextState)
            gScore(nextState)=tentativeG;
            parent(nextState)=uint32(currentState);
            h=abs(nextRow-goalRow)+abs(nextColumn-goalColumn);
            pushHeap(uint32(nextState),tentativeG+h+1e-6*h);
            if captureTrace
                newlyDiscovered(end+1,:)=[nextRow nextColumn]; %#ok<AGROW>
            end
        end
    end
    if captureTrace
        trace.newCells{end+1,1}=unique(newlyDiscovered,'rows');
    end
end
if goalState==0
    info.message='Khong ton tai duong di A* cai tien.';
    info.trace=trace;
    return;
end

reverseStates=zeros(cellCount*2,1,'uint32');
count=1; reverseStates(count)=uint32(goalState); state=goalState;
while state~=startState
    state=double(parent(state));
    if state==0,error('Loi noi bo khi truy vet A* cai tien.');end
    count=count+1;
    if count>numel(reverseStates),reverseStates(end+cellCount,1)=uint32(0);end
    reverseStates(count)=uint32(state);
end
states=double(flipud(reverseStates(1:count)));
cells=zeros(count,1);
for i=1:count,[cells(i),~]=decodeState(states(i));end
[pathRows,pathColumns]=ind2sub([rows columns],cells);
pathMeters=gridToWorld(pathRows,pathColumns,resolution);
pathDirections=sign(diff(pathMeters,1,1));
if size(pathDirections,1)>1
    numberOfTurns=sum(any(diff(pathDirections,1,1)~=0,2));
else
    numberOfTurns=0;
end
info.success=true;
info.pathCostCells=gScore(goalState);
info.message='OK';
info.numberOfTurns=numberOfTurns;
info.trace=trace;

    function index=stateIndex(cellIndex,direction)
        index=cellIndex+direction*cellCount;
    end
    function [cellIndex,direction]=decodeState(index)
        direction=floor((index-1)/cellCount);
        cellIndex=index-direction*cellCount;
    end
    function pushHeap(stateId,priority)
        heapSize=heapSize+1;
        if heapSize>numel(heapStates)
            heapStates(end+stateCount,1)=uint32(0);
            heapPriority(end+stateCount,1)=inf;
        end
        index=heapSize;
        while index>1
            parentIndex=floor(index/2);
            if heapPriority(parentIndex)<=priority,break;end
            heapStates(index)=heapStates(parentIndex);
            heapPriority(index)=heapPriority(parentIndex);
            index=parentIndex;
        end
        heapStates(index)=stateId; heapPriority(index)=priority;
    end
    function stateId=popHeap()
        stateId=heapStates(1);
        lastState=heapStates(heapSize); lastPriority=heapPriority(heapSize);
        heapSize=heapSize-1;
        if heapSize==0,return;end
        index=1;
        while true
            left=2*index;
            if left>heapSize,break;end
            right=left+1; child=left;
            if right<=heapSize && heapPriority(right)<heapPriority(left),child=right;end
            if heapPriority(child)>=lastPriority,break;end
            heapStates(index)=heapStates(child);
            heapPriority(index)=heapPriority(child);
            index=child;
        end
        heapStates(index)=lastState; heapPriority(index)=lastPriority;
    end
end
