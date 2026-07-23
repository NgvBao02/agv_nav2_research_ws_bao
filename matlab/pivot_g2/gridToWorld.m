function point = gridToWorld(row, column, resolution)
%GRIDTOWORLD Doi chi so cell sang toa do tam cell (m).
point = [(double(column(:)) - 0.5) * resolution, ...
         (double(row(:)) - 0.5) * resolution];
end
