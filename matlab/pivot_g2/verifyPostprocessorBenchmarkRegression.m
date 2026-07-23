function report = verifyPostprocessorBenchmarkRegression()
%VERIFYPOSTPROCESSORBENCHMARKREGRESSION Smoke/regression cua pipeline moi.
config=defaultCornerOptimizerConfig();config.enablePlots=false;
config.enableAnimation=false;config.capturePlannerTrace=false;
comparison=defaultPostprocessorComparisonConfig(config);
comparison.enableAnimation=false;comparison.saveFigures=false;
assert(~comparison.arcPreprocessing.lineOfSightPruning, ...
    'Benchmark khong duoc LOS-prune rieng proposed/fixed-radius.');
maps=createMapSuite(config);

% Ca thang khong corner: moi method phai tao reference va khong loi schema.
straightMap=maps(5);scenario=straightMap.startGoalPairs(1);
result=runPathPostprocessorComparison('THETA_STAR',straightMap,scenario, ...
    config,comparison);
assert(height(result.resultTable)==6,'Phai co dung 6 method.');
assert(result.fairness.passed&&result.fairness.sameInputPathForAll, ...
    'Fairness check that bai.');
assert(all(result.resultTable.PostprocessSuccess), ...
    'Co postprocessor that bai tren regression case.');
assert(numel(unique(result.resultTable.InputPathSignature))==1, ...
    'InputPathSignature khong dong nhat.');

% Kiem tra Savitzky-Golay bao toan hang so va endpoint.
synthetic=[linspace(0,2,15).' 0.2*sin(linspace(0,2*pi,15).')];
[smooth,info]=nav2SavitzkyGolaySmootherEquivalent(synthetic,straightMap, ...
    config,comparison);
assert(norm(smooth(1,:)-synthetic(1,:))<1e-12&& ...
    norm(smooth(end,:)-synthetic(end,:))<1e-12,'SG lam doi endpoint.');
assert(abs(sum(info.coefficients)-1)<1e-10, ...
    'He so SG khong bao toan tin hieu hang.');

% Profile arc trai-phai lien tiep khong duoc tron dau omega.
path=[0 0;0.5 0;1 0.2;1.5 0;2 0];
reference=buildContinuousReferenceFromPath(path,config,comparison);
assert(any(reference.omega>0)&&any(reference.omega<0), ...
    'Profile omega mat mot chieu cong tai inflection.');

report=struct('passed',true,'methods', ...
    {result.resultTable.Postprocessor},'fairness',result.fairness, ...
    'savitzkyGolayCoefficientSum',sum(info.coefficients), ...
    'leftRightArcProfilePassed',true);
fprintf('POSTPROCESSOR REGRESSION: PASS (%d methods)\n', ...
    height(result.resultTable));
end
