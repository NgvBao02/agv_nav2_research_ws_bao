function [planner,planningOccupancy] = planPostprocessorInputPath( ...
        plannerName,map,scenario,config,comparison)
%PLANPOSTPROCESSORINPUTPATH Chay planner duy nhat cho moi ca benchmark.
plannerName=upper(char(plannerName));
planningOccupancy=inflateOccupancyGrid(map.occupancy, ...
    config.inflationRadius,map.resolution);
potential=computeCostmapPotential(planningOccupancy,map.resolution, ...
    comparison.costDecayDistance);
capture=comparison.captureSearchTrace;
switch plannerName
    case 'NAVFN_DIJKSTRA'
        planner=nav2GridPlanner(planningOccupancy,scenario.start,scenario.goal, ...
            map.resolution,plannerName,potential,0,capture);
        planner.plugin='nav2_navfn_planner::NavfnPlanner(use_astar=false)';
    case 'NAVFN_ASTAR'
        planner=nav2GridPlanner(planningOccupancy,scenario.start,scenario.goal, ...
            map.resolution,plannerName,potential,0,capture);
        planner.plugin='nav2_navfn_planner::NavfnPlanner(use_astar=true)';
    case 'SMAC_2D'
        planner=nav2GridPlanner(planningOccupancy,scenario.start,scenario.goal, ...
            map.resolution,plannerName,potential,comparison.smac2DCostWeight,capture);
        planner.plugin='nav2_smac_planner::SmacPlanner2D';
    case 'THETA_STAR'
        planner=thetaStarPlannerEquivalent(planningOccupancy,scenario.start, ...
            scenario.goal,map.resolution,potential,comparison.thetaCostWeight,capture);
    case {'TURN_ASTAR','IMPROVED_TURN_PENALTY'}
        timer=tic;
        [path,info]=planGridPath(planningOccupancy,scenario.start, ...
            scenario.goal,config,capture);
        heading=pathHeading(path);
        planner=struct('name','TURN_ASTAR','plugin','CUSTOM_TURN_ASTAR', ...
            'implementation','PROPOSED_PLANNER','success',info.success, ...
            'path',path,'poses',[path heading],'modes',{{}},'radii',[], ...
            'trace',info.trace,'planningTime',toc(timer), ...
            'expandedNodes',info.expandedNodes,'pathCost',info.pathCostCells, ...
            'message',info.message);
    otherwise
        error(['Planner %s khong duoc ho tro trong benchmark hau xu ly. ' ...
            'Dung THETA_STAR, NAVFN_ASTAR, NAVFN_DIJKSTRA, SMAC_2D hoac TURN_ASTAR.'], ...
            plannerName);
end
if ~planner.success
    error('Planner %s that bai: %s',plannerName,planner.message);
end
if size(planner.path,1)<2
    error('Planner %s tra ve it hon hai diem.',plannerName);
end
end

function theta=pathHeading(path)
if size(path,1)<2,theta=0;return;end
theta=atan2(diff(path(:,2)),diff(path(:,1)));
theta=[theta;theta(end)];
end
