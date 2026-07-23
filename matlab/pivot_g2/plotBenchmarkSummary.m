function plotBenchmarkSummary(allResults, representatives, config)
%PLOTBENCHMARKSUMMARY Hinh theo map, representative, arc rate va chuan hoa.
valid = ~isnan(allResults.CompletionTime);
data = allResults(valid,:);
mapNames = unique(string(data.MapName),'stable');
methods = ["PIVOT_ONLY","FIXED_RADIUS","ADAPTIVE_PIVOT_OR_ARC"];
colors = lines(3);

figure('Name','REPRESENTATIVE_PATHS','Color','w');
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for k = 1:numel(representatives)
    nexttile;
    if isempty(representatives{k}), axis off; continue; end
    result=representatives{k}; drawOccupancyMap(result.map);
    for m=1:3
        sim=result.methodResults(m).simulation;
        plot(sim.x,sim.y,'Color',colors(m,:),'LineWidth',1.1);
    end
    adaptive=result.methodResults(3).decisions;
    for q=1:numel(adaptive)
        if strcmp(adaptive(q).selectedType,'ARC'), marker='o'; color=[0 0.6 0];
        else, marker='x'; color=[0.85 0.1 0.1]; end
        plot(adaptive(q).corner.vertex(1),adaptive(q).corner.vertex(2), ...
            marker,'Color',color,'LineWidth',1.5);
    end
    title(result.map.name,'Interpreter','none','FontSize',9);
end
sgtitle('Representative paths: o = adaptive arc, x = adaptive pivot');

metricNames={'CompletionTime','NumberOfFullStops','PositionRMSE', ...
    'MinimumClearance','Jomega','Success'};
metricTitles={'Completion time (s)','Full stops','Position RMSE (m)', ...
    'Minimum clearance (m)','Jomega (rad/s)','Success rate'};
figure('Name','PERFORMANCE_BY_MAP','Color','w');
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for q=1:numel(metricNames)
    matrix=groupMean(data,mapNames,methods,metricNames{q});
    nexttile; bar(matrix); grid on;
    set(gca,'XTick',1:numel(mapNames),'XTickLabel',mapNames, ...
        'XTickLabelRotation',25);
    ylabel(metricTitles{q});
    if q==1, legend({'Pivot','Fixed','Adaptive'},'Location','best'); end
end
sgtitle('PERFORMANCE BY MAP');

figure('Name','ARC_SELECTION_RATE','Color','w');
arcMatrix=groupMean(data,mapNames,methods,'ArcSelectionRate');
bar(arcMatrix); grid on;
set(gca,'XTick',1:numel(mapNames),'XTickLabel',mapNames,'XTickLabelRotation',25);
ylabel('Ty le goc chon arc'); ylim([0 1]);
legend({'Pivot','Fixed','Adaptive'},'Location','best');
title('ARC SELECTION RATE');

figure('Name','NORMALIZED_METRICS','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
normalized={'TimePerMeter','StopsPerCorner','JomegaPerMeter', ...
    'OptimizationTimePerCorner'};
normalizedTitles={'Time/m (s/m)','Stops/corner','Jomega/m', ...
    'Optimization time/corner (s)'};
for q=1:4
    nexttile;
    bar(groupMean(data,mapNames,methods,normalized{q})); grid on;
    set(gca,'XTick',1:numel(mapNames),'XTickLabel',mapNames, ...
        'XTickLabelRotation',25);
    ylabel(normalizedTitles{q});
end
sgtitle('NORMALIZED METRICS');

if config.saveFigures
    if ~exist(config.outputDirectory,'dir'),mkdir(config.outputDirectory);end
    figures=findall(groot,'Type','figure');
    for i=1:numel(figures)
        exportgraphics(figures(i),fullfile(config.outputDirectory, ...
            sprintf('figure_%02d.png',figures(i).Number)),'Resolution',160);
    end
end
end

function matrix=groupMean(data,mapNames,methods,field)
matrix=nan(numel(mapNames),numel(methods));
for i=1:numel(mapNames)
    for j=1:numel(methods)
        mask=string(data.MapName)==mapNames(i) & string(data.Method)==methods(j);
        matrix(i,j)=mean(data.(field)(mask),'omitnan');
    end
end
end
