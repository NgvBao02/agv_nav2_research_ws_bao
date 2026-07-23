function [path,info] = planGridPath(occupancy,startPoint,goalPoint,config,captureTrace)
%PLANGRIDPATH Chon A* truyen thong hoac A* cai tien bang config.
if nargin<5,captureTrace=false;end
mode=upper(char(config.plannerMode));
switch mode
    case 'TRADITIONAL_ASTAR'
        [path,info]=astar4Direction(occupancy,startPoint,goalPoint, ...
            config.resolution,captureTrace);
    case 'IMPROVED_TURN_PENALTY'
        penalty=computeAStarTurnPenalty(config);
        [path,info]=astar4DirectionTurnPenalty(occupancy,startPoint,goalPoint, ...
            config.resolution,penalty, ...
            config.improvedAStar.forbidImmediateReverse,captureTrace);
    otherwise
        error('plannerMode khong hop le: %s',config.plannerMode);
end
end
