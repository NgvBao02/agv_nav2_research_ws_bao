function scenario = createScenario(name, startPoint, goalPoint)
%CREATESCENARIO Tao mot cap start-goal co ten.
validateattributes(startPoint, {'numeric'}, {'vector','numel',2,'finite'});
validateattributes(goalPoint, {'numeric'}, {'vector','numel',2,'finite'});
scenario = struct('name', char(name), 'start', double(startPoint(:).'), ...
    'goal', double(goalPoint(:).'));
end
