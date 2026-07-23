function reducedPath = removeCollinearPoints(pathMeters)
%REMOVECOLLINEARPOINTS Giu start, goal va cac diem doi huong.
validateattributes(pathMeters, {'numeric'}, {'2d','ncols',2,'finite'});
if size(pathMeters,1) <= 2
    reducedPath = pathMeters;
    return;
end
step = diff(pathMeters,1,1);
step(abs(step) < 1e-12) = 0;
keep = true(size(pathMeters,1),1);
for i = 2:size(pathMeters,1)-1
    crossValue = step(i-1,1)*step(i,2) - step(i-1,2)*step(i,1);
    sameDirection = dot(step(i-1,:),step(i,:)) > 0;
    keep(i) = ~(abs(crossValue) < 1e-12 && sameDirection);
end
reducedPath = pathMeters(keep,:);
end
