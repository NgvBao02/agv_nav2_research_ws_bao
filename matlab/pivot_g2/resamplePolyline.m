function pathOut = resamplePolyline(pathIn,spacing)
%RESAMPLEPOLYLINE Noi suy path theo chieu dai cung, giu chinh xac hai dau.
validateattributes(pathIn,{'numeric'},{'2d','ncols',2,'finite'});
validateattributes(spacing,{'numeric'},{'scalar','positive','finite'});
if size(pathIn,1)<2,error('Path phai co it nhat hai diem.');end
keep=[true;hypot(diff(pathIn(:,1)),diff(pathIn(:,2)))>1e-12];
path=pathIn(keep,:);
if size(path,1)<2,error('Path bi trung toan bo diem.');end
s=[0;cumsum(hypot(diff(path(:,1)),diff(path(:,2))))];
samples=(0:spacing:s(end)).';
if isempty(samples)||samples(end)<s(end)-1e-12,samples(end+1,1)=s(end);end
pathOut=[interp1(s,path(:,1),samples,'linear'), ...
    interp1(s,path(:,2),samples,'linear')];
pathOut(1,:)=path(1,:);pathOut(end,:)=path(end,:);
end
