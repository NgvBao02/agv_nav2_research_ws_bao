function comparison = defaultNav2ComparisonConfig(config)
%DEFAULTNAV2COMPARISONCONFIG Tham so bo so sanh cac ho global planner Nav2.
comparison.plannerNames = {'NAVFN_DIJKSTRA','NAVFN_ASTAR','SMAC_2D', ...
    'THETA_STAR','SMAC_HYBRID','SMAC_STATE_LATTICE','PROPOSED_ADAPTIVE'};
comparison.captureSearchTrace = true;
comparison.gridConnectivity = 8;
comparison.smac2DCostWeight = 2.0;
comparison.costDecayDistance = 0.60;
comparison.thetaCostWeight = 0.8;
comparison.se2.angleBins = 16;
comparison.se2.minimumTurningRadius = 0.40;
comparison.se2.latticeRadii = [0.30 0.50];
comparison.se2.primitiveLength = max(config.resolution*sqrt(2),0.28);
comparison.se2.primitiveSampleSpacing = 0.04;
comparison.se2.goalPositionTolerance = 0.24;
comparison.se2.goalHeadingToleranceBins = 1;
comparison.se2.maxIterations = 150000;
comparison.se2.allowReverse = false;
% Tat Laplacian mac dinh: no thay doi hinh hoc goc cua planner va pha rang
% buoc G2 cua proposed. Chi bat nhu mot ablation chung, khong mang claim G2.
comparison.commonSmoother.enabled = false;
comparison.commonSmoother.cornerMethod = 'ADAPTIVE_PIVOT_OR_ARC';
comparison.commonSmoother.iterations = 35;
comparison.commonSmoother.smoothWeight = 0.32;
comparison.commonSmoother.dataWeight = 0.12;
comparison.commonSmoother.relaxation = 0.55;
comparison.commonSmoother.maximumDisplacement = 0.08;
comparison.commonSmoother.validationStride = 1;
comparison.animationFrameTime = 0.50;
comparison.animationPlaybackSpeed = 10;
comparison.animationVideoFrameRate = 20;
comparison.enableAnimation = true;
comparison.saveAnimation = true;
comparison.saveFigures = true;
comparison.outputRoot = fullfile(config.outputDirectory,'nav2_comparison');
comparison.timestampedFolder = true;
end
