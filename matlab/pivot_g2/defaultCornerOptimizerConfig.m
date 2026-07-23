function config = defaultCornerOptimizerConfig()
%DEFAULTCORNEROPTIMIZERCONFIG Tao toan bo tham so mac dinh cua mo phong.
config.algorithmRevision = 'PIVOT_G2_COMMON_WINDOW_V2';

% Ban do va che do chay
config.resolution = 0.20;                 % m/cell
config.runMode = 'single';               % 'single' hoac 'batch'
config.singleMapIndex = 1;
config.singleScenarioIndex = 1;
config.numRepeats = 10;
config.randomSeed = 2026;
config.enablePlots = true;
config.enableAnimation = true;
config.enablePlannerAnimation = true;
config.enableRobotAnimation = true;
config.animationSkip = 5;
config.animationPlaybackSpeed = 4.0;       % 4x thoi gian thuc
config.plannerAnimationSkip = 4;
config.plannerAnimationPause = 0.015;
config.plannerMode = 'IMPROVED_TURN_PENALTY'; % hoac 'TRADITIONAL_ASTAR'
config.improvedAStar.penaltyMode = 'PAPER';   % 'PAPER' hoac 'DYNAMIC'
% K2=1.94 la TONG chi phi tuong doi cua buoc quay 90 do roi di mot cell.
% Phan phat cong them ma A* su dung la lambda=K2-1=0.94.
config.improvedAStar.paperTurnCostRatio = 1.94;
config.improvedAStar.forbidImmediateReverse = true;
config.capturePlannerTrace = true;
config.saveFigures = false;
config.outputDirectory = fullfile(pwd, 'results');

% Thong so robot vi sai
config.robot.length = 0.40;               % m
config.robot.width = 0.30;                % m
config.robot.wheelBase = 0.24;            % m
config.robot.maxLinearSpeed = 0.35;       % m/s
config.robot.maxAngularSpeed = 1.20;      % rad/s
config.robot.maxLinearAcceleration = 0.35;% m/s^2
config.robot.maxLinearDeceleration = 0.45;% m/s^2
config.robot.maxAngularAcceleration = 1.50;% rad/s^2
config.robot.maxWheelSpeed = 0.50;        % m/s, duoc kiem tra cho tung lenh
config.robot.clearanceSafe = 0.05;        % m tu footprint den vat can
% Ho so nay chi la mo phong cua ban thao, chua phai tham so robot ROS 2 that.
% Truoc benchmark phan cung phai thay bang ket qua do va doi ten profile.
config.robot.profileName = 'PAPER_SIMULATION_UNVERIFIED';
config.robot.measured = false;

% A* va hinh hoc
config.planningSafetyMargin = 0.05;        % du phong cho sai so bam quy dao
config.inflationRadius = hypot(config.robot.length / 2, ...
    config.robot.width / 2) + config.robot.clearanceSafe + ...
    config.planningSafetyMargin;
config.arcRadiusCandidates = [0.20 0.25 0.30 0.35 0.40 0.50 0.60];
config.fixedRadius = 0.30;
config.deltaTimeSelection = 0.15;         % s
config.maxCornerRadiusFraction = 0.45;    % tranh hai cung ke nhau bi chong lan
% Pivot va G2 duoc so san tren cung mot cua so quanh goc. Toc do bien la
% toc do mong muon o ngoai cua so; time parameterizer se tu ha toc neu cua
% so ngan, va tinh them thoi gian chuyen tiep o ngoai cua so.
config.timeComparison.windowFraction = config.maxCornerRadiusFraction;
config.timeComparison.boundarySpeed = config.robot.maxLinearSpeed;
% Sau khi loc cac cung nhanh hon pivot, xem cac cung trong khoang nay la
% canh tranh ve thoi gian va chon phuong an ben vung hon.
config.adaptiveSelection.timeCompetitiveSlack = 0.20; % s
config.adaptiveSelection.clearanceWeight = 0.35;
config.adaptiveSelection.angularRateWeight = 0.25;
config.adaptiveSelection.curvatureEnergyWeight = 0.40;
% Chuan hoa theo moc vat ly co dinh, khong min-max theo tap ung vien tai goc.
config.adaptiveSelection.clearanceScale = hypot(config.robot.length/2, ...
    config.robot.width/2);
config.adaptiveSelection.angularRateScale = config.robot.maxAngularSpeed;
config.adaptiveSelection.curvatureEnergyScale = ...
    (pi/2)/config.fixedRadius;
% Quintic Bezier co curvature=0 tai hai dau, dung lam doan chuyen G2 cho
% phuong phap de xuat. Fixed-radius baseline van giu cung tron thuong.
config.adaptiveSelection.curvatureTransitionEnabled = true;
config.adaptiveSelection.bezierControlFraction = 0.35;
config.arcSampleSpacing = 0.02;           % m
config.straightSampleSpacing = 0.02;      % m
config.pivotAngleStep = pi / 90;          % 2 do
config.geometrySampleStep = 0.04;         % m
config.numericTolerance = 1e-9;

% Speed profile va mo phong
config.dt = 0.02;                         % s
config.controller.lookAheadTime = 0.65;   % s, lookahead lon hon de tranh dao dong waypoint
config.controller.minimumLookAheadDistance = 0.22; % m
config.controller.maximumLookAheadDistance = 0.55; % m
config.controller.minimumArcLookAheadDistance = 0.10; % m
config.controller.arcLookAheadRadiusFraction = 0.60;
config.controller.progressSearchWindow = 25;       % so mau reference tim projection
config.controller.pointTolerance = 0.025; % m de tang chi so reference
config.controller.cornerPositionTolerance = 0.020;
config.controller.pivotAngleTolerance = 2*pi/180;
config.controller.kLongitudinal = 1.4;
config.controller.kLateral = 4.0;
config.controller.kHeading = 2.2;
config.controller.kPivotPosition = 0.8;
config.controller.headingSlowdown = 1.5;
% Feed-forward curvature tai phep chieu gan nhat; preview chi dung tao sai
% so doc. Dieu nay tranh quay som khi curvature thay doi tren transition G2.
config.controller.feedforwardAtProjection = true;
config.controller.goalPositionTolerance = 0.06;
config.controller.goalHeadingTolerance = 4 * pi / 180;
config.controller.goalHoldTime = 0.20;
config.controller.extraSimulationTime = 8.0;
config.controller.stationaryThreshold = 0.01;

% Bao cao va benchmark
config.verbose = true;
config.checkpointAfterEachMap = true;
config.plotRepresentativeScenarioIndex = 1;
end
