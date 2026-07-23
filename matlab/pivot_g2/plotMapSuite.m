function plotMapSuite(maps)
%PLOTMAPSUITE Tong quan 6 ban do va tat ca cap start-goal.
figure('Name','MAP_SUITE_OVERVIEW','Color','w');
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
colors = lines(max(arrayfun(@(m)numel(m.startGoalPairs),maps)));
for k = 1:numel(maps)
    nexttile;
    drawOccupancyMap(maps(k));
    for j = 1:numel(maps(k).startGoalPairs)
        s = maps(k).startGoalPairs(j);
        plot([s.start(1) s.goal(1)],[s.start(2) s.goal(2)],':', ...
            'Color',colors(j,:),'LineWidth',0.8);
        plot(s.start(1),s.start(2),'o','Color',colors(j,:), ...
            'MarkerFaceColor',colors(j,:),'MarkerSize',4);
        plot(s.goal(1),s.goal(2),'x','Color',colors(j,:),'LineWidth',1.2);
    end
    title(sprintf('%s (%.0f x %.0f m)',maps(k).name,maps(k).width, ...
        maps(k).height),'Interpreter','none','FontSize',9);
end
sgtitle('MAP SUITE: o = start, x = goal');
end
