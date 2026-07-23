function figureHandles = plotNav2ComparisonResults(comparisonResult,outputDirectory,comparison)
%PLOTNAV2COMPARISONRESULTS Tach planner, reference, actual va metric.
runs=comparisonResult.runs;map=comparisonResult.map;colors=lines(numel(runs));
valid=arrayfun(@(r)~isempty(fieldnames(r.simulation)),runs);
rawValid=arrayfun(@(r)isfield(r.planner,'path')&&~isempty(r.planner.path),runs);
figureHandles=gobjects(4,1);

figureHandles(1)=createPathFigure('Raw global-planner paths', ...
    sprintf('%s - %s: raw planner paths',map.name,comparisonResult.scenario.name), ...
    @(run)[run.planner.path(:,1) run.planner.path(:,2)],rawValid);
figureHandles(2)=createPathFigure('Common-smoothed references', ...
    sprintf('%s - %s: common-smoothed references',map.name,comparisonResult.scenario.name), ...
    @(run)[run.reference.x run.reference.y],valid);
figureHandles(3)=createPathFigure('Actual robot trajectories', ...
    sprintf('%s - %s: actual robot trajectories',map.name,comparisonResult.scenario.name), ...
    @(run)[run.simulation.x run.simulation.y],valid);

T=comparisonResult.resultTable(valid,:);labels=strrep(string(T.Planner),'_',' ');
figureHandles(4)=figure('Name','Nav2 planner metric comparison','Color','w', ...
    'Position',[100 70 1450 800]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
fields={'PlanningTime','CompletionTime','ActualPathLength', ...
    'NumberOfFullStops','PositionRMSE','MinimumClearance'};
titles={'Planning time (s)','Completion time (s)','Actual path (m)', ...
    'Full stops','Position RMSE (m)','Minimum clearance (m)'};
for q=1:numel(fields)
    nexttile;bar(T.(fields{q}),'FaceColor','flat','CData',colors(valid,:));grid on;
    set(gca,'XTick',1:height(T),'XTickLabel',labels,'XTickLabelRotation',28);
    ylabel(titles{q});
end
sgtitle('Nav2-style global planner comparison (same smoother and controller)');

if comparison.saveFigures
    exportgraphics(figureHandles(1),fullfile(outputDirectory, ...
        'comparison_planner_paths.png'),'Resolution',180);
    exportgraphics(figureHandles(2),fullfile(outputDirectory, ...
        'comparison_reference_paths.png'),'Resolution',180);
    exportgraphics(figureHandles(3),fullfile(outputDirectory, ...
        'comparison_actual_trajectories.png'),'Resolution',180);
    % Ten cu duoc giu de cac script phan tich truoc day khong bi hong.
    exportgraphics(figureHandles(3),fullfile(outputDirectory, ...
        'comparison_paths.png'),'Resolution',180);
    exportgraphics(figureHandles(4),fullfile(outputDirectory, ...
        'comparison_metrics.png'),'Resolution',180);
end

    function handle=createPathFigure(windowName,titleText,pathGetter,mask)
        handle=figure('Name',windowName,'Color','w','Position',[80 80 1250 760]);
        drawOccupancyMap(map);grid on;
        for plannerIndex=1:numel(runs)
            if ~mask(plannerIndex),continue;end
            path=pathGetter(runs(plannerIndex));
            plot(path(:,1),path(:,2),'Color',colors(plannerIndex,:), ...
                'LineWidth',1.6,'DisplayName', ...
                strrep(runs(plannerIndex).planner.name,'_',' '));
        end
        plot(comparisonResult.scenario.start(1),comparisonResult.scenario.start(2), ...
            'go','MarkerFaceColor','g','MarkerSize',8,'DisplayName','Start');
        plot(comparisonResult.scenario.goal(1),comparisonResult.scenario.goal(2), ...
            'rp','MarkerFaceColor','r','MarkerSize',11,'DisplayName','Goal');
        legend('Location','bestoutside');title(titleText,'Interpreter','none');
    end
end
