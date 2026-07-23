function comparison = defaultPostprocessorComparisonConfig(config)
%DEFAULTPOSTPROCESSORCOMPARISONCONFIG Cau hinh benchmark bo hau xu ly.
% Moi phuong phap nhan cung mot path cua planner va dung cung controller.

comparison.plannerName = 'THETA_STAR';
comparison.postprocessorNames = {'NO_SMOOTHER','NAV2_SIMPLE', ...
    'NAV2_SAVITZKY_GOLAY','NAV2_CONSTRAINED', ...
    'FIXED_RADIUS_ARC','PROPOSED_PIVOT_ARC'};
comparison.captureSearchTrace = false;
comparison.costDecayDistance = 0.60;
comparison.smac2DCostWeight = 2.0;
comparison.thetaCostWeight = 0.8;

% Gia tri mac dinh tu nav2_smoother::SimpleSmoother.
comparison.simple.tolerance = 1e-10;
comparison.simple.maxIterations = 1000;
comparison.simple.dataWeight = 0.20;
comparison.simple.smoothWeight = 0.30;
comparison.simple.doRefinement = true;
comparison.simple.refinementCount = 2;
comparison.simple.enforcePathInversion = true;

% Gia tri mac dinh tu nav2_smoother::SavitzkyGolaySmoother.
comparison.savitzkyGolay.windowSize = 7;
comparison.savitzkyGolay.polynomialOrder = 3;
comparison.savitzkyGolay.doRefinement = true;
comparison.savitzkyGolay.refinementCount = 2;
comparison.savitzkyGolay.enforcePathInversion = true;

% MATLAB-equivalent cua Constrained Smoother. Nav2 goc toi uu bang Ceres;
% cac trong so duoi day giu dung cac nhom muc tieu: distance, smoothness,
% obstacle va curvature, nhung khong duoc goi la plugin C++ goc.
comparison.constrained.maxIterations = 120;
comparison.constrained.tolerance = 1e-5;
comparison.constrained.stepSize = 0.12;
comparison.constrained.dataWeight = 0.18;
comparison.constrained.smoothWeight = 0.42;
comparison.constrained.obstacleWeight = 0.22;
comparison.constrained.curvatureWeight = 0.18;
comparison.constrained.maximumDisplacement = 0.35;
comparison.constrained.targetClearance = ...
    config.robot.clearanceSafe + config.planningSafetyMargin;
comparison.constrained.minimumTurningRadius = 0.20;
comparison.constrained.validationStride = 2;

% Bo chuyen path hinh hoc thanh reference chung cho cac smoother dang XY.
comparison.reference.sampleSpacing = config.straightSampleSpacing;
comparison.reference.straightCurvatureThreshold = 0.08; % 1/m
comparison.reference.maximumCurvature = 1/0.12;
comparison.reference.headingFilterWindow = 5;
% Khong cho rieng proposed/fixed-radius rut gon line-of-sight: nhu vay moi
% method thuc su nhan cung mot polyline hinh hoc tu planner. Collinear
% reduction van duoc phep vi khong thay doi polyline.
comparison.arcPreprocessing.lineOfSightPruning = false;

% Lua chon cung thich nghi: chi danh doi mot khoang thoi gian nho, sau do
% uu tien clearance va toc do goc thap hon. Cac trong so khong co don vi.
comparison.proposed.timeCompetitiveSlack = 0.20; % s
comparison.proposed.clearanceWeight = 0.35;
comparison.proposed.angularRateWeight = 0.25;
comparison.proposed.curvatureEnergyWeight = 0.40;
comparison.proposed.curvatureTransitionEnabled = true;
comparison.proposed.bezierControlFraction = 0.35;
comparison.proposed.clearanceScale = ...
    config.adaptiveSelection.clearanceScale;
comparison.proposed.angularRateScale = ...
    config.adaptiveSelection.angularRateScale;
comparison.proposed.curvatureEnergyScale = ...
    config.adaptiveSelection.curvatureEnergyScale;
% Laplacian micro-refinement dich chuyen cac mau rieng le, vi vay khong bao
% toan rang buoc endpoint/tangent/curvature cua Bezier G2. Tat trong phuong
% phap chinh; chi bat cho ablation va khong duoc goi output sau do la G2.
comparison.proposedRefinement.enabled = false;
comparison.proposedRefinement.iterations = 20;
comparison.proposedRefinement.smoothWeight = 0.25;
comparison.proposedRefinement.dataWeight = 0.08;
comparison.proposedRefinement.relaxation = 0.55;
comparison.proposedRefinement.maximumDisplacement = 0.03;
comparison.proposedRefinement.validationStride = 1;

comparison.animationFrameTime = 0.50;
comparison.animationPlaybackSpeed = 20;
comparison.animationVideoFrameRate = 20;
comparison.enableAnimation = true;
comparison.saveAnimation = true;
comparison.saveFigures = true;
comparison.closeFiguresAfterExport = false;
comparison.outputRoot = fullfile(config.outputDirectory, ...
    'postprocessor_comparison');
comparison.batchOutputRoot = fullfile(config.outputDirectory, ...
    'postprocessor_benchmark');
end
