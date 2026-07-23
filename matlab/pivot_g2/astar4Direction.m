function [pathMeters, info] = astar4Direction(occupancy, startPoint, goalPoint, resolution, captureTrace)
%ASTAR4DIRECTION A* bon huong voi heuristic Manhattan, khong dung toolbox.
validateattributes(occupancy, {'numeric','logical'}, {'2d','nonempty'});
validateattributes(startPoint, {'numeric'}, {'vector','numel',2,'finite'});
validateattributes(goalPoint, {'numeric'}, {'vector','numel',2,'finite'});
validateattributes(resolution, {'numeric'}, {'scalar','positive','finite'});
if nargin < 5, captureTrace = false; end
trace = struct('currentCells',zeros(0,2),'newCells',{cell(0,1)}, ...
    'planner','TRADITIONAL_ASTAR');

occupancy = logical(occupancy);
[rows, columns] = size(occupancy);
startColumn = floor(startPoint(1) / resolution) + 1;
startRow = floor(startPoint(2) / resolution) + 1;
goalColumn = floor(goalPoint(1) / resolution) + 1;
goalRow = floor(goalPoint(2) / resolution) + 1;

info = struct('success', false, 'expandedNodes', 0, 'pathCostCells', inf, ...
    'message', '', 'startCell', [startRow startColumn], ...
    'goalCell', [goalRow goalColumn], 'trace',trace, ...
    'planner','TRADITIONAL_ASTAR','turnPenalty',0,'turnCostRatio',1, ...
    'numberOfTurns',nan);
pathMeters = zeros(0,2);

if startRow < 1 || startRow > rows || startColumn < 1 || startColumn > columns
    info.message = 'Start nam ngoai ban do.';
    return;
end
if goalRow < 1 || goalRow > rows || goalColumn < 1 || goalColumn > columns
    info.message = 'Goal nam ngoai ban do.';
    return;
end
if occupancy(startRow,startColumn) || occupancy(goalRow,goalColumn)
    info.message = 'Start hoac goal nam trong vat can.';
    return;
end

nodeCount = rows * columns;
startNode = sub2ind([rows columns], startRow, startColumn);
goalNode = sub2ind([rows columns], goalRow, goalColumn);
gScore = inf(nodeCount,1);
parent = zeros(nodeCount,1,'uint32');
closed = false(nodeCount,1);
gScore(startNode) = 0;

% Heap cho phep ban sao khi giam khoa; node da closed se duoc bo qua.
heapNodes = zeros(max(64, nodeCount),1,'uint32');
heapPriority = inf(max(64, nodeCount),1);
heapSize = 0;
startH = abs(startRow-goalRow) + abs(startColumn-goalColumn);
pushHeap(uint32(startNode), startH);
directions = [0 1; 1 0; 0 -1; -1 0];

found = false;
while heapSize > 0
    current = double(popHeap());
    if closed(current)
        continue;
    end
    closed(current) = true;
    info.expandedNodes = info.expandedNodes + 1;
    [currentRow,currentColumn] = ind2sub([rows columns], current);
    newlyDiscovered = zeros(0,2);
    if captureTrace
        trace.currentCells(end+1,:) = [currentRow currentColumn];
    end
    if current == goalNode
        if captureTrace, trace.newCells{end+1,1}=newlyDiscovered; end
        found = true;
        break;
    end
    for d = 1:4
        nextRow = currentRow + directions(d,1);
        nextColumn = currentColumn + directions(d,2);
        if nextRow < 1 || nextRow > rows || nextColumn < 1 || nextColumn > columns
            continue;
        end
        if occupancy(nextRow,nextColumn)
            continue;
        end
        nextNode = sub2ind([rows columns], nextRow, nextColumn);
        tentativeG = gScore(current) + 1;
        if tentativeG + eps < gScore(nextNode)
            gScore(nextNode) = tentativeG;
            parent(nextNode) = uint32(current);
            h = abs(nextRow-goalRow) + abs(nextColumn-goalColumn);
            % Tie-break rat nho uu tien node gan goal, van giu f = g+h.
            pushHeap(uint32(nextNode), tentativeG + h + 1e-6*h);
            if captureTrace
                newlyDiscovered(end+1,:)=[nextRow nextColumn]; %#ok<AGROW>
            end
        end
    end
    if captureTrace
        trace.newCells{end+1,1}=unique(newlyDiscovered,'rows');
    end
end

if ~found
    info.message = 'Khong ton tai duong di A* bon huong.';
    return;
end

reverseNodes = zeros(round(gScore(goalNode))+1,1,'uint32');
count = 1;
reverseNodes(count) = uint32(goalNode);
node = goalNode;
while node ~= startNode
    node = double(parent(node));
    if node == 0
        error('Loi noi bo khi truy vet duong A*.');
    end
    count = count + 1;
    reverseNodes(count) = uint32(node);
end
nodes = double(flipud(reverseNodes(1:count)));
[pathRows,pathColumns] = ind2sub([rows columns], nodes);
pathMeters = gridToWorld(pathRows, pathColumns, resolution);
info.success = true;
info.pathCostCells = gScore(goalNode);
info.message = 'OK';
directionsPath = sign(diff(pathMeters,1,1));
if size(directionsPath,1)>1
    info.numberOfTurns = sum(any(diff(directionsPath,1,1)~=0,2));
else
    info.numberOfTurns = 0;
end
info.trace = trace;

    function pushHeap(nodeId, priority)
        heapSize = heapSize + 1;
        if heapSize > numel(heapNodes)
            heapNodes(end+max(64,nodeCount),1) = uint32(0);
            heapPriority(end+max(64,nodeCount),1) = inf;
        end
        index = heapSize;
        while index > 1
            parentIndex = floor(index/2);
            if heapPriority(parentIndex) <= priority
                break;
            end
            heapNodes(index) = heapNodes(parentIndex);
            heapPriority(index) = heapPriority(parentIndex);
            index = parentIndex;
        end
        heapNodes(index) = nodeId;
        heapPriority(index) = priority;
    end

    function nodeId = popHeap()
        nodeId = heapNodes(1);
        lastNode = heapNodes(heapSize);
        lastPriority = heapPriority(heapSize);
        heapSize = heapSize - 1;
        if heapSize == 0
            return;
        end
        index = 1;
        while true
            left = 2*index;
            if left > heapSize
                break;
            end
            right = left + 1;
            child = left;
            if right <= heapSize && heapPriority(right) < heapPriority(left)
                child = right;
            end
            if heapPriority(child) >= lastPriority
                break;
            end
            heapNodes(index) = heapNodes(child);
            heapPriority(index) = heapPriority(child);
            index = child;
        end
        heapNodes(index) = lastNode;
        heapPriority(index) = lastPriority;
    end
end
