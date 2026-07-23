function clearance = computeMinimumClearance(pose, map, robot)
%COMPUTEMINIMUMCLEARANCE Khoang cach Euclid nho nhat footprint-vat can.
polygon = transformRobotFootprint(pose,robot);
if isfield(map,'obstacleRows') && isfield(map,'obstacleColumns')
    obstacleRows = map.obstacleRows;
    obstacleColumns = map.obstacleColumns;
else
    [obstacleRows,obstacleColumns] = find(map.occupancy);
end
if isempty(obstacleRows)
    clearance = inf;
    return;
end
r = map.resolution;
if isfield(map,'obstacleCenters')
    centers = map.obstacleCenters;
else
    centers = gridToWorld(obstacleRows,obstacleColumns,r);
end
centerDistance = hypot(centers(:,1)-pose(1),centers(:,2)-pose(2));
[~,nearestIndex] = min(centerDistance);
clearance = distanceToCell(polygon,obstacleRows(nearestIndex), ...
    obstacleColumns(nearestIndex),r);

robotRadius = hypot(robot.length/2,robot.width/2);
cellRadius = r/sqrt(2);
lowerBound = max(0,centerDistance-robotRadius-cellRadius);
candidateIndices = find(lowerBound <= clearance + 1e-12);
for k = 1:numel(candidateIndices)
    i = candidateIndices(k);
    d = distanceToCell(polygon,obstacleRows(i),obstacleColumns(i),r);
    if d < clearance
        clearance = d;
        if clearance <= 1e-12
            clearance = 0;
            return;
        end
    end
end
end

function distance = distanceToCell(polygon,row,column,resolution)
x0 = (column-1)*resolution;
y0 = (row-1)*resolution;
rectangle = [x0 y0; x0+resolution y0; ...
    x0+resolution y0+resolution; x0 y0+resolution];
if localIntersect(polygon,rectangle)
    distance = 0;
    return;
end
distance = inf;
for i = 1:4
    a1 = polygon(i,:); a2 = polygon(mod(i,4)+1,:);
    for j = 1:4
        b1 = rectangle(j,:); b2 = rectangle(mod(j,4)+1,:);
        distance = min(distance,segmentDistance(a1,a2,b1,b2));
    end
end
end

function d = segmentDistance(a,b,c,dPoint)
d = min([pointSegmentDistance(a,c,dPoint), ...
         pointSegmentDistance(b,c,dPoint), ...
         pointSegmentDistance(c,a,b), ...
         pointSegmentDistance(dPoint,a,b)]);
end

function d = pointSegmentDistance(point,a,b)
ab = b-a;
t = dot(point-a,ab) / max(dot(ab,ab),eps);
t = min(1,max(0,t));
projection = a + t*ab;
d = norm(point-projection);
end

function intersects = localIntersect(a,b)
axes = [localNormals(a); localNormals(b)];
intersects = true;
for k = 1:size(axes,1)
    pa = a*axes(k,:).'; pb = b*axes(k,:).';
    if max(pa) < min(pb)-1e-12 || max(pb) < min(pa)-1e-12
        intersects = false;
        return;
    end
end
end

function normals = localNormals(p)
e = p([2:end 1],:)-p;
normals = [-e(:,2) e(:,1)];
normals = normals ./ max(hypot(normals(:,1),normals(:,2)),eps);
end
