function [penalty,details] = computeAStarTurnPenalty(config)
%COMPUTEASTARTURNPENALTY Tinh phan chi phi tang them khi doi huong.
% A* luon cong chi phi co so 1 cho mot buoc di. Ham nay tra ve lambda,
% trong do chi phi buoc re la 1+lambda. K2 la tong ty le chi phi cua
% buoc re, vi vay lambda=K2-1, khong phai lambda=K2.
mode = upper(char(config.improvedAStar.penaltyMode));
switch mode
    case 'PAPER'
        if isfield(config.improvedAStar,'paperTurnCostRatio')
            turnCostRatio = config.improvedAStar.paperTurnCostRatio;
        elseif isfield(config.improvedAStar,'paperTurnPenalty')
            % Tuong thich voi cau hinh cu: gia tri 1.94 thuc chat la K2,
            % tuc tong ty le chi phi, khong phai phan phat tang them.
            turnCostRatio = config.improvedAStar.paperTurnPenalty;
        else
            error(['Thieu improvedAStar.paperTurnCostRatio ', ...
                '(K2 cua bai bao, mac dinh 1.94).']);
        end
        validateattributes(turnCostRatio,{'numeric'}, ...
            {'scalar','finite','>=',1},mfilename,'paperTurnCostRatio');
        penalty = turnCostRatio-1;
        straightTime = nan;
        turnTime = nan;
    case 'DYNAMIC'
        validateattributes(config.resolution,{'numeric'}, ...
            {'scalar','positive','finite'},mfilename,'config.resolution');
        validateattributes(config.robot.maxLinearSpeed,{'numeric'}, ...
            {'scalar','positive','finite'},mfilename, ...
            'config.robot.maxLinearSpeed');
        straightTime = config.resolution/config.robot.maxLinearSpeed;
        [~,pivotDetails] = estimatePivotTime(pi/2,0,0,config.robot);
        turnTime = pivotDetails.rotationTime;
        penalty = turnTime/straightTime;
        turnCostRatio = 1+penalty;
    otherwise
        error('improvedAStar.penaltyMode phai la PAPER hoac DYNAMIC.');
end
validateattributes(penalty,{'numeric'},{'scalar','nonnegative','finite'});
details = struct('mode',mode,'straightStepCost',1, ...
    'turnCostRatio',turnCostRatio,'additiveTurnPenalty',penalty, ...
    'straightTime',straightTime,'turnTime',turnTime);
end
