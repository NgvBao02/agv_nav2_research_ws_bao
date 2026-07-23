function validation = validatePostprocessedPath(path,map,config,spacing)
%VALIDATEPOSTPROCESSEDPATH Kiem tra footprint lien tuc tren path XY.
if nargin<4||isempty(spacing),spacing=config.straightSampleSpacing;end
dense=resamplePolyline(path,min(spacing,config.geometrySampleStep));
theta=pathTangentHeadings(dense);
validation=evaluatePoseSequenceSafety([dense theta],map,config);
end

function theta=pathTangentHeadings(path)
n=size(path,1);delta=zeros(n,2);
delta(1,:)=path(2,:)-path(1,:);delta(end,:)=path(end,:)-path(end-1,:);
if n>2,delta(2:end-1,:)=path(3:end,:)-path(1:end-2,:);end
theta=unwrap(atan2(delta(:,2),delta(:,1)));
end
