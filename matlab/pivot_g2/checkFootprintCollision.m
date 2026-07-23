function [collision, vertices] = checkFootprintCollision(pose, map, robot, ~)
%CHECKFOOTPRINTCOLLISION Kiem tra giao polygon-cell bang Separating Axis.
% Cach kiem tra nay xet toan bo hinh chu nhat, khong chi tam robot.
vertices = transformRobotFootprint(pose, robot);
if any(vertices(:,1) < 0) || any(vertices(:,1) > map.width) || ...
        any(vertices(:,2) < 0) || any(vertices(:,2) > map.height)
    collision = true;
    return;
end

r = map.resolution;
[rowMin,colMin] = worldToGrid([min(vertices(:,1)) min(vertices(:,2))],map);
[rowMax,colMax] = worldToGrid([max(vertices(:,1)) max(vertices(:,2))],map);
rowMin = max(1,rowMin); rowMax = min(size(map.occupancy,1),rowMax);
colMin = max(1,colMin); colMax = min(size(map.occupancy,2),colMax);
[obstacleRows,obstacleColumns] = find(map.occupancy(rowMin:rowMax,colMin:colMax));
obstacleRows = obstacleRows + rowMin - 1;
obstacleColumns = obstacleColumns + colMin - 1;
collision = false;
for i = 1:numel(obstacleRows)
    x0 = (obstacleColumns(i)-1)*r;
    y0 = (obstacleRows(i)-1)*r;
    rectangle = [x0 y0; x0+r y0; x0+r y0+r; x0 y0+r];
    if convexPolygonsIntersect(vertices,rectangle)
        collision = true;
        return;
    end
end
end

function intersects = convexPolygonsIntersect(a,b)
axes = [edgeNormals(a); edgeNormals(b)];
intersects = true;
for k = 1:size(axes,1)
    axis = axes(k,:);
    if norm(axis) < eps
        continue;
    end
    projectionA = a*axis.';
    projectionB = b*axis.';
    if max(projectionA) < min(projectionB)-1e-12 || ...
            max(projectionB) < min(projectionA)-1e-12
        intersects = false;
        return;
    end
end
end

function normals = edgeNormals(polygon)
edges = polygon([2:end 1],:) - polygon;
normals = [-edges(:,2) edges(:,1)];
lengths = hypot(normals(:,1),normals(:,2));
normals = normals ./ max(lengths,eps);
end
