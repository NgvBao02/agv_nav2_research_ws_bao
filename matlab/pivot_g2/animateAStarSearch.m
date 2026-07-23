function animateAStarSearch(map,trace,finalPath,scenario,config)
%ANIMATEASTARSEARCH Minh hoa OPEN, CLOSED, current va duong cuoi cua A*.
if isempty(trace) || isempty(trace.currentCells)
    warning('Animation:NoTrace','Khong co trace tim kiem de tao animation.');
    return;
end
[rows,columns]=size(map.occupancy);
openMask=false(rows,columns);
closedMask=false(rows,columns);
figure('Name',['A* search animation - ' trace.planner],'Color','w');
drawOccupancyMap(map); grid on;
openHandle=scatter(nan,nan,18,[0.2 0.75 0.25],'filled', ...
    'DisplayName','OPEN');
closedHandle=scatter(nan,nan,18,[0.2 0.45 0.95],'filled', ...
    'DisplayName','CLOSED');
currentHandle=scatter(nan,nan,60,[1.0 0.55 0.05],'filled', ...
    'MarkerEdgeColor','k','DisplayName','Current');
plot(scenario.start(1),scenario.start(2),'go','MarkerFaceColor','g', ...
    'MarkerSize',8,'DisplayName','Start');
plot(scenario.goal(1),scenario.goal(2),'rp','MarkerFaceColor','r', ...
    'MarkerSize',11,'DisplayName','Goal');
pathHandle=plot(nan,nan,'y-','LineWidth',3,'DisplayName','Final path');
legend('Location','bestoutside');
titleHandle=title('');

for k=1:size(trace.currentCells,1)
    if k<=numel(trace.newCells)
        discovered=trace.newCells{k};
        for j=1:size(discovered,1)
            if ~closedMask(discovered(j,1),discovered(j,2))
                openMask(discovered(j,1),discovered(j,2))=true;
            end
        end
    end
    current=trace.currentCells(k,:);
    openMask(current(1),current(2))=false;
    closedMask(current(1),current(2))=true;
    if mod(k-1,config.plannerAnimationSkip)==0 || ...
            k==size(trace.currentCells,1)
        [openRows,openColumns]=find(openMask);
        [closedRows,closedColumns]=find(closedMask);
        openPoints=gridToWorld(openRows,openColumns,map.resolution);
        closedPoints=gridToWorld(closedRows,closedColumns,map.resolution);
        currentPoint=gridToWorld(current(1),current(2),map.resolution);
        set(openHandle,'XData',openPoints(:,1),'YData',openPoints(:,2));
        set(closedHandle,'XData',closedPoints(:,1),'YData',closedPoints(:,2));
        set(currentHandle,'XData',currentPoint(1),'YData',currentPoint(2));
        set(titleHandle,'String',sprintf('%s | buoc %d/%d | OPEN=%d CLOSED=%d', ...
            trace.planner,k,size(trace.currentCells,1),nnz(openMask),nnz(closedMask)), ...
            'Interpreter','none');
        drawnow;
        if config.plannerAnimationPause>0,pause(config.plannerAnimationPause);end
    end
end
set(pathHandle,'XData',finalPath(:,1),'YData',finalPath(:,2));
set(titleHandle,'String',sprintf('%s hoan tat: %d node duong di', ...
    trace.planner,size(finalPath,1)),'Interpreter','none');
drawnow;
end
