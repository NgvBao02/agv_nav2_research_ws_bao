function pruned = prunePathLineOfSight(path,occupancy,map)
%PRUNEPATHLINEOFSIGHT Rut gon waypoint bang supercover line-of-sight.
% Grid da inflate duoc dung de khong danh doi an toan lay duong ngan.
if size(path,1)<=2,pruned=path;return;end
[rows,columns,valid]=worldToGrid(path,map);
if ~all(valid),error('Path co diem nam ngoai map.');end
n=size(path,1);indices=1;anchor=1;
while anchor<n
    target=n;
    while target>anchor+1&&~visible(rows(anchor),columns(anchor), ...
            rows(target),columns(target),occupancy)
        target=target-1;
    end
    indices(end+1,1)=target; %#ok<AGROW>
    anchor=target;
end
pruned=path(indices,:);

    function tf=visible(r0,c0,r1,c1,grid)
        count=max(abs(r1-r0),abs(c1-c0))+1;
        lineRows=round(linspace(r0,r1,count));
        lineColumns=round(linspace(c0,c1,count));
        ids=sub2ind(size(grid),lineRows,lineColumns);
        tf=~any(grid(ids));
        if ~tf,return;end
        for q=2:numel(lineRows)
            if lineRows(q)~=lineRows(q-1)&& ...
                    lineColumns(q)~=lineColumns(q-1)
                if grid(lineRows(q-1),lineColumns(q))|| ...
                        grid(lineRows(q),lineColumns(q-1))
                    tf=false;return;
                end
            end
        end
    end
end
