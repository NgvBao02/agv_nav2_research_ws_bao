function config = applyMeasuredRobotProfile(config,measured)
%APPLYMEASUREDROBOTPROFILE Gan ho so robot do that va tinh lai tham so phu thuoc.
required={'profileName','length','width','wheelBase','maxLinearSpeed', ...
    'maxAngularSpeed','maxLinearAcceleration','maxLinearDeceleration', ...
    'maxAngularAcceleration','maxWheelSpeed','clearanceSafe'};
missing=required(~isfield(measured,required));
if ~isempty(missing)
    error('RobotProfile:MissingField','Thieu field: %s',strjoin(missing,', '));
end
positive=required(2:end-1);
for i=1:numel(positive)
    value=measured.(positive{i});
    validateattributes(value,{'numeric'},{'scalar','finite','positive'}, ...
        mfilename,positive{i});
end
validateattributes(measured.clearanceSafe,{'numeric'}, ...
    {'scalar','finite','nonnegative'},mfilename,'clearanceSafe');
if isempty(char(measured.profileName))
    error('RobotProfile:EmptyName','profileName khong duoc rong.');
end
for i=1:numel(required)
    config.robot.(required{i})=measured.(required{i});
end
config.robot.profileName=char(config.robot.profileName);
config.robot.measured=true;
config.inflationRadius=hypot(config.robot.length/2,config.robot.width/2)+ ...
    config.robot.clearanceSafe+config.planningSafetyMargin;
config.timeComparison.boundarySpeed=config.robot.maxLinearSpeed;
config.adaptiveSelection.clearanceScale= ...
    hypot(config.robot.length/2,config.robot.width/2);
config.adaptiveSelection.angularRateScale=config.robot.maxAngularSpeed;
end
