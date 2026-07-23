function decision = selectCornerManeuver(corner, map, config, method)
%SELECTCORNERMANEUVER Chon pivot, cung co dinh hoac cung thich nghi.
method = upper(char(method));
pivot = generatePivotManeuver(corner,map,config);
switch method
    case 'PIVOT_ONLY'
        radii = zeros(1,0);
    case 'FIXED_RADIUS'
        radii = config.fixedRadius;
    case 'ADAPTIVE_PIVOT_OR_ARC'
        radii = config.arcRadiusCandidates;
    otherwise
        error('Phuong phap khong hop le: %s',method);
end

if isempty(radii)
    candidates = generateArcCandidates(corner,zeros(1,0),map,config);
elseif strcmp(method,'ADAPTIVE_PIVOT_OR_ARC') && ...
        isfield(config,'adaptiveSelection') && ...
        isfield(config.adaptiveSelection,'curvatureTransitionEnabled') && ...
        config.adaptiveSelection.curvatureTransitionEnabled
    candidates = generateTransitionCurveCandidates(corner,radii,map,config);
else
    candidates = generateArcCandidates(corner,radii,map,config);
end
[pivot,candidates]=estimateCornerManeuverTimes(corner,pivot,candidates,config);
validIndices = find([candidates.valid]);
bestArcIndex = nan;
bestArcTime = inf;
selectionScore = nan;
competitiveCount = 0;
if ~isempty(validIndices)
    [bestArcTime,localIndex] = min([candidates(validIndices).predictedTime]);
    bestArcIndex = validIndices(localIndex);
end

selectedType = 'PIVOT';
selectedRadius = 0;
reason = 'Phuong an pivot bat buoc.';
if strcmp(method,'FIXED_RADIUS') && ~isempty(validIndices)
    selectedType = 'ARC';
    selectedRadius = candidates(bestArcIndex).radius;
    reason = 'Cung ban kinh co dinh hop le.';
elseif strcmp(method,'FIXED_RADIUS')
    reason = 'Cung co dinh khong hop le; fallback sang pivot.';
elseif strcmp(method,'ADAPTIVE_PIVOT_OR_ARC') && ~isempty(validIndices) && ...
        bestArcTime + config.deltaTimeSelection < pivot.predictedTime
    % Hai tang: (1) cung phai nhanh hon pivot mot muc co y nghia; (2) trong
    % cac cung gan toi uu ve thoi gian, uu tien clearance, |omega| va
    % nang luong do cong thap.
    % Cach nay loai bo tinh trang moi goc deu mac dinh R=0.30 m chi vi no
    % nhanh hon vai phan tram giay, dong thoi van co gioi han danh doi ro.
    if isfield(config,'adaptiveSelection')
        adaptive=config.adaptiveSelection;
    else
        adaptive=struct('timeCompetitiveSlack',0.20, ...
            'clearanceWeight',0.35,'angularRateWeight',0.25, ...
            'curvatureEnergyWeight',0.40, ...
            'clearanceScale',hypot(config.robot.length/2,config.robot.width/2), ...
            'angularRateScale',config.robot.maxAngularSpeed, ...
            'curvatureEnergyScale',(pi/2)/config.fixedRadius);
    end
    competitive=validIndices([candidates(validIndices).predictedTime] <= ...
        bestArcTime+adaptive.timeCompetitiveSlack+config.numericTolerance);
    competitiveCount=numel(competitive);
    clearance=[candidates(competitive).minimumClearance];
    angularRate=abs([candidates(competitive).omega]);
    if isfield(candidates,'curvatureEnergy')
        curvatureEnergy=[candidates(competitive).curvatureEnergy];
    else
        curvatureEnergy=abs(corner.turnAngle)./[candidates(competitive).radius];
    end
    clearanceScore=clamp01((clearance-config.robot.clearanceSafe)./ ...
        max(adaptive.clearanceScale,eps));
    angularScore=1-clamp01(angularRate./ ...
        max(adaptive.angularRateScale,eps));
    curvatureScore=1-clamp01(curvatureEnergy./ ...
        max(adaptive.curvatureEnergyScale,eps));
    scores=adaptive.clearanceWeight*clearanceScore + ...
        adaptive.angularRateWeight*angularScore + ...
        adaptive.curvatureEnergyWeight*curvatureScore;
    [selectionScore,selectedLocal]=max(scores);
    bestArcIndex=competitive(selectedLocal);
    selectedType = 'ARC';
    selectedRadius = candidates(bestArcIndex).radius;
    reason = sprintf(['Cung nhanh hon pivot; chon trong %d ung vien canh ' ...
        'tranh theo clearance, |omega| va curvature energy.'],competitiveCount);
elseif strcmp(method,'ADAPTIVE_PIVOT_OR_ARC') && isempty(validIndices)
    reason = 'Khong co cung an toan; chon pivot.';
elseif strcmp(method,'ADAPTIVE_PIVOT_OR_ARC')
    reason = 'Loi ich thoi gian cua cung khong du delta_T; chon pivot.';
end

if strcmp(selectedType,'ARC')
    selectedClearance = candidates(bestArcIndex).minimumClearance;
    selectedTime = candidates(bestArcIndex).predictedTime;
    selectedPoses = candidates(bestArcIndex).poses;
    valid = candidates(bestArcIndex).valid;
else
    selectedClearance = pivot.minimumClearance;
    selectedTime = pivot.predictedTime;
    selectedPoses = pivot.poses;
    valid = pivot.valid;
end
decision = struct('corner',corner,'method',method, ...
    'selectedType',selectedType,'selectedRadius',selectedRadius, ...
    'selectedTime',selectedTime,'selectedClearance',selectedClearance, ...
    'selectedPoses',selectedPoses,'valid',valid,'reason',reason, ...
    'pivot',pivot,'arcCandidates',candidates, ...
    'bestArcIndex',bestArcIndex,'pivotTime',pivot.predictedTime, ...
    'bestArcTime',bestArcTime,'rejectedArcCandidates', ...
    sum(~[candidates.valid]),'selectionScore',selectionScore, ...
    'competitiveArcCandidates',competitiveCount);
end

function value=clamp01(value)
value=min(1,max(0,value));
end
