function handles = plotPostprocessorComparison(result,outputDirectory,comparison)
%PLOTPOSTPROCESSORCOMPARISON Reference va actual tach rieng, kem panel.
runs=result.runs;map=result.map;colors=lines(numel(runs));
valid=arrayfun(@(r)~isempty(fieldnames(r.simulation)),runs);
handles=gobjects(5,1);

handles(1)=figure('Name','Shared planner input','Color','w', ...
    'Position',[90 90 1150 720]);drawOccupancyMap(map);grid on;
plot(result.sharedInputPath(:,1),result.sharedInputPath(:,2),'b-', ...
    'LineWidth',2.0,'DisplayName','Planner path (shared)');
drawStartGoal(result);legend('Location','bestoutside');
title(sprintf('%s - %s/%s: ONE shared planner path', ...
    result.planner.name,map.name,result.scenario.name),'Interpreter','none');

handles(2)=overlayFigure('Postprocessed reference paths',false);
handles(3)=overlayFigure('Actual robot trajectories',true);

handles(4)=figure('Name','Six synchronized result panels','Color','w', ...
    'Position',[30 40 1600 900]);layout=tiledlayout(2,3, ...
    'TileSpacing','compact','Padding','compact');
for i=1:numel(runs)
    nexttile(layout);drawOccupancyMap(map);grid on;
    if valid(i)
        plot(runs(i).reference.x,runs(i).reference.y,'--', ...
            'Color',[0.45 0.45 0.45],'LineWidth',1.0);
        plot(runs(i).simulation.x,runs(i).simulation.y,'-', ...
            'Color',colors(i,:),'LineWidth',1.6);
        title(sprintf('%s | T=%.1fs | RMSE=%.3fm', ...
            runs(i).method.displayName,runs(i).simulation.time(end), ...
            result.resultTable.PositionRMSE(i)),'Interpreter','none');
    else,title([runs(i).method.displayName ' | FAILED'],'Interpreter','none');end
end
sgtitle(layout,sprintf('%s - %s/%s: gray reference, color actual', ...
    result.planner.name,map.name,result.scenario.name),'Interpreter','none');

T=result.resultTable(valid,:);labels=strrep(string(T.Postprocessor),'_',' ');
handles(5)=figure('Name','Postprocessor metrics','Color','w', ...
    'Position',[50 50 1550 850]);metricLayout=tiledlayout(2,3, ...
    'TileSpacing','compact','Padding','compact');
fields={'PostprocessTime','CompletionTime','IntegratedSquaredCurvature', ...
    'NumberOfFullStops','PositionRMSE','MinimumClearance'};
titles={'Postprocess time (s)','Completion time (s)', ...
    'Integral curvature^2 (1/m)','Full stops','Position RMSE (m)', ...
    'Minimum clearance (m)'};
for q=1:numel(fields)
    nexttile(metricLayout);bar(T.(fields{q}),'FaceColor','flat', ...
        'CData',colors(valid,:));grid on;
    set(gca,'XTick',1:height(T),'XTickLabel',labels, ...
        'XTickLabelRotation',25);ylabel(titles{q});
end
sgtitle(metricLayout,'Fixed planner + fixed controller; only postprocessor changes');

if comparison.saveFigures
    names={'input_planner_path.png','comparison_reference_paths.png', ...
        'comparison_actual_trajectories.png','comparison_method_panels.png', ...
        'comparison_metrics.png'};
    for i=1:numel(handles)
        exportgraphics(handles(i),fullfile(outputDirectory,names{i}), ...
            'Resolution',180);
    end
end
if comparison.closeFiguresAfterExport,close(handles);end

    function handle=overlayFigure(windowName,useActual)
        handle=figure('Name',windowName,'Color','w','Position',[80 80 1300 760]);
        drawOccupancyMap(map);grid on;
        for methodIndex=1:numel(runs)
            if ~valid(methodIndex),continue;end
            if useActual
                x=runs(methodIndex).simulation.x;y=runs(methodIndex).simulation.y;
            else
                x=runs(methodIndex).reference.x;y=runs(methodIndex).reference.y;
            end
            plot(x,y,'Color',colors(methodIndex,:),'LineWidth',1.7, ...
                'DisplayName',runs(methodIndex).method.displayName);
        end
        drawStartGoal(result);legend('Location','bestoutside');
        title(sprintf('%s - %s/%s: %s',result.planner.name,map.name, ...
            result.scenario.name,windowName),'Interpreter','none');
    end
end

function drawStartGoal(result)
plot(result.scenario.start(1),result.scenario.start(2),'go', ...
    'MarkerFaceColor','g','MarkerSize',8,'DisplayName','Start');
plot(result.scenario.goal(1),result.scenario.goal(2),'rp', ...
    'MarkerFaceColor','r','MarkerSize',11,'DisplayName','Goal');
end
