function plotSimulationResults(result, config)
%PLOTSIMULATIONRESULTS Tao 9 figure bat buoc cho che do single.
map = result.map;
colors = lines(3);
labels = {'Pivot-only','Fixed-radius','Adaptive'};

figure('Name','Figure 1 - A* paths','Color','w');
drawOccupancyMap(map);
plot(result.rawPath(:,1),result.rawPath(:,2),':','Color',[0.2 0.55 1], ...
    'LineWidth',1.4,'DisplayName','A* tho');
plot(result.reducedPath(:,1),result.reducedPath(:,2),'mo-', ...
    'LineWidth',1.5,'MarkerSize',5,'DisplayName','Waypoint tinh gon');
plot(result.rawPath(1,1),result.rawPath(1,2),'go','MarkerFaceColor','g', ...
    'DisplayName','Start');
plot(result.rawPath(end,1),result.rawPath(end,2),'rp','MarkerFaceColor','r', ...
    'MarkerSize',11,'DisplayName','Goal');
title(sprintf('%s - %s',map.name,result.scenario.name),'Interpreter','none');
legend('Location','bestoutside'); grid on;

figure('Name','Figure 2 - Corner candidates','Color','w');
drawOccupancyMap(map);
adaptive = result.methodResults(3);
if ~isempty(adaptive.decisions)
    decision = adaptive.decisions(1);
    candidateHandles = gobjects(0); candidateLabels = {};
    for i = 1:numel(decision.arcCandidates)
        candidate = decision.arcCandidates(i);
        if isempty(candidate.poses)
            displayArc = generateQuarterCircleArc(decision.corner, ...
                candidate.radius,config.arcSampleSpacing);
            poses = displayArc.poses;
        else
            poses = candidate.poses;
        end
        if candidate.valid, color=[0.1 0.65 0.2]; style='-';
        else, color=[0.85 0.2 0.2]; style='--'; end
        h = plot(poses(:,1),poses(:,2),style,'Color',color,'LineWidth',1.1);
        candidateHandles(end+1)=h; %#ok<AGROW>
        candidateLabels{end+1}=sprintf('R=%.2f: %s',candidate.radius, ...
            ternary(candidate.valid,'hop le','loai')); %#ok<AGROW>
    end
    selected = decision.selectedPoses;
    hSelected = plot(selected(:,1),selected(:,2),'b-','LineWidth',3);
    sampleIndices = unique(round(linspace(1,size(selected,1),4)));
    for index = sampleIndices
        vertices = transformRobotFootprint(selected(index,:),config.robot);
        patch(vertices(:,1),vertices(:,2),[0.2 0.55 1], ...
            'FaceAlpha',0.16,'EdgeColor',[0 0.25 0.8]);
    end
    plot(decision.corner.vertex(1),decision.corner.vertex(2),'ko', ...
        'MarkerFaceColor','y','MarkerSize',7);
    candidateHandles(end+1)=hSelected;
    candidateLabels{end+1}=sprintf('Chon %s',decision.selectedType);
    legend(candidateHandles,candidateLabels,'Location','bestoutside');
    margin = max(config.arcRadiusCandidates)+0.7;
    xlim(decision.corner.vertex(1)+[-margin margin]);
    ylim(decision.corner.vertex(2)+[-margin margin]);
    title(sprintf('Chi tiet goc 1: %s',decision.reason),'Interpreter','none');
else
    text(map.width/2,map.height/2,'Khong co goc cua','HorizontalAlignment','center');
end
grid on;

figure('Name','Figure 3 - Trajectory comparison','Color','w');
drawOccupancyMap(map);
for m = 1:3
    simulation = result.methodResults(m).simulation;
    plot(simulation.x,simulation.y,'Color',colors(m,:),'LineWidth',1.5, ...
        'DisplayName',labels{m});
end
legend('Location','bestoutside'); grid on;
title('Quy dao thuc cua ba phuong phap');

plotTimeSeries(4,'Van toc thang','Thoi gian (s)','v (m/s)','v');
plotTimeSeries(5,'Van toc goc','Thoi gian (s)','omega (rad/s)','omega');

figure('Name','Figure 6 - Wheel velocities','Color','w'); hold on; grid on;
for m = 1:3
    simulation=result.methodResults(m).simulation;
    plot(simulation.time,simulation.leftWheelVelocity,'-', ...
        'Color',colors(m,:),'LineWidth',1.1,'DisplayName',[labels{m} ' - trai']);
    plot(simulation.time,simulation.rightWheelVelocity,'--', ...
        'Color',colors(m,:),'LineWidth',1.1,'DisplayName',[labels{m} ' - phai']);
end
yline(config.robot.maxWheelSpeed,'k:','Gioi han banh');
yline(-config.robot.maxWheelSpeed,'k:');
xlabel('Thoi gian (s)'); ylabel('Van toc banh (m/s)');
title('Van toc banh trai/phai'); legend('Location','bestoutside');

plotTimeSeries(7,'Sai so bam duong','Thoi gian (s)','Sai so vi tri (m)', ...
    'positionError');
clearanceFigure=plotTimeSeries(8,'Clearance theo thoi gian','Thoi gian (s)', ...
    'Clearance (m)','clearance');
figure(clearanceFigure); yline(config.robot.clearanceSafe,'r--','Clearance an toan');

figure('Name','Figure 9 - Metric comparison','Color','w');
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
metrics = {'CompletionTime','NumberOfFullStops','PositionRMSE', ...
    'MinimumClearance','Jv','Jomega'};
metricTitles = {'Thoi gian (s)','So lan dung','RMSE vi tri (m)', ...
    'Clearance min (m)','J_v (m/s)','J_omega (rad/s)'};
for q = 1:6
    nexttile;
    values = arrayfun(@(x)x.metrics.(metrics{q}),result.methodResults);
    bar(values,'FaceColor','flat','CData',colors); grid on;
    set(gca,'XTick',1:3,'XTickLabel',{'Pivot','Fixed','Adaptive'}, ...
        'XTickLabelRotation',20);
    ylabel(metricTitles{q});
end
sgtitle('So sanh chi so chinh');

    function figureHandle=plotTimeSeries(number,name,xLabel,yLabel,field)
        figureHandle=figure('Name',sprintf('Figure %d - %s',number,name),'Color','w');
        hold on; grid on;
        for methodIndex=1:3
            simulation=result.methodResults(methodIndex).simulation;
            plot(simulation.time,simulation.(field),'Color',colors(methodIndex,:), ...
                'LineWidth',1.25,'DisplayName',labels{methodIndex});
        end
        xlabel(xLabel); ylabel(yLabel); title(name); legend('Location','best');
    end
end

function value = ternary(condition,trueValue,falseValue)
if condition, value=trueValue; else, value=falseValue; end
end
