function vertices = transformRobotFootprint(pose, robot)
%TRANSFORMROBOTFOOTPRINT Doi bon dinh footprint tu he robot sang he world.
validateattributes(pose, {'numeric'}, {'vector','numel',3,'finite'});
required = {'length','width'};
if ~all(isfield(robot,required))
    error('robot phai co truong length va width.');
end
halfLength = robot.length / 2;
halfWidth = robot.width / 2;
bodyVertices = [ halfLength  halfWidth; ...
                 halfLength -halfWidth; ...
                -halfLength -halfWidth; ...
                -halfLength  halfWidth];
c = cos(pose(3)); s = sin(pose(3));
rotation = [c -s; s c];
vertices = bodyVertices * rotation.' + pose(1:2);
end
