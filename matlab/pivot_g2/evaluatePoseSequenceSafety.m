function result = evaluatePoseSequenceSafety(poses,map,config)
%EVALUATEPOSESEQUENCESAFETY Kiem tra footprint tren noi suy SE(2) day du.
% Moi cap pose duoc chen mau sao cho ca dich chuyen tam va cung quet do quay
% cua dinh footprint khong vuot qua buoc hinh hoc cho phep.
validateattributes(poses,{'numeric'},{'2d','ncols',3,'finite'});
if isempty(poses)
    result=struct('safe',false,'collision',true,'minimumClearance',nan, ...
        'failedIndex',0,'samplesChecked',0,'denseSampleCount',0);
    return;
end

maximumStep=min(config.geometrySampleStep,map.resolution/2);
if maximumStep<=0,error('geometrySampleStep va map resolution phai duong.');end
sweepRadius=hypot(config.robot.length/2,config.robot.width/2);
dense=poses(1,:);
for i=1:size(poses,1)-1
    translation=norm(poses(i+1,1:2)-poses(i,1:2));
    rotation=wrapAngle(poses(i+1,3)-poses(i,3));
    subdivisions=max([1,ceil(translation/maximumStep), ...
        ceil(abs(rotation)*sweepRadius/maximumStep)]);
    fraction=(1:subdivisions).'/subdivisions;
    xy=poses(i,1:2)+fraction.*(poses(i+1,1:2)-poses(i,1:2));
    theta=poses(i,3)+fraction*rotation;
    dense=[dense;xy theta]; %#ok<AGROW>
end

minimumClearance=inf;failedIndex=0;
for i=1:size(dense,1)
    collision=checkFootprintCollision(dense(i,:),map,config.robot, ...
        config.geometrySampleStep);
    clearance=computeMinimumClearance(dense(i,:),map,config.robot);
    minimumClearance=min(minimumClearance,clearance);
    if collision||clearance<config.robot.clearanceSafe-1e-9
        failedIndex=i;break;
    end
end
if failedIndex>0,samplesChecked=failedIndex;else,samplesChecked=size(dense,1);end
result=struct('safe',failedIndex==0,'collision',failedIndex>0, ...
    'minimumClearance',minimumClearance,'failedIndex',failedIndex, ...
    'samplesChecked',samplesChecked,'denseSampleCount',size(dense,1));
end
