function animationFile = animateNav2Comparison(comparisonResult,outputDirectory,comparison)
%ANIMATENAV2COMPARISON Chay dong thoi tat ca robot trong mot cua so.
runs=comparisonResult.runs;map=comparisonResult.map;runCount=numel(runs);
valid=arrayfun(@(r)~isempty(fieldnames(r.simulation)),runs);colors=lines(runCount);
figureHandle=figure('Name','Multi-planner execution animation','Color','w', ...
    'Position',[20 40 1700 900]);
layout=tiledlayout(2,4,'TileSpacing','compact','Padding','compact');
axesHandles=gobjects(runCount,1);trailHandles=gobjects(runCount,1);
robotHandles=gobjects(runCount,1);headingHandles=gobjects(runCount,1);
for i=1:runCount
    axesHandles(i)=nexttile(layout);axes(axesHandles(i)); %#ok<LAXES>
    drawOccupancyMap(map);grid on;
    if valid(i)
        plot(runs(i).reference.x,runs(i).reference.y,'-', ...
            'Color',[0.65 0.65 0.65],'LineWidth',0.8);
    end
    trailHandles(i)=plot(nan,nan,'-','Color',colors(i,:),'LineWidth',1.4);
    robotHandles(i)=patch(nan,nan,colors(i,:),'FaceAlpha',0.38, ...
        'EdgeColor',colors(i,:),'LineWidth',1.2);
    headingHandles(i)=plot(nan,nan,'r-','LineWidth',1.4);
    title(strrep(runs(i).planner.name,'_',' '),'FontSize',9);
end
nexttile(layout,8);axis off;
text(0,0.9,'Same physical time','FontWeight','bold','FontSize',12);
text(0,0.72,'Gray: common-smoothed reference');
text(0,0.60,'Color: actual trajectory');
text(0,0.48,'Rectangle: robot footprint');
text(0,0.30,sprintf('Playback: %.1fx',comparison.animationPlaybackSpeed));
sgtitle(layout,sprintf('%s - %s',map.name,comparisonResult.scenario.name), ...
    'Interpreter','none');
maximumTime=max(arrayfun(@(r)lastTime(r),runs));
animationTimes=(0:comparison.animationFrameTime:maximumTime).';
if animationTimes(end)<maximumTime,animationTimes(end+1)=maximumTime;end
animationFile='';video=[];
if comparison.saveAnimation
    animationFile=fullfile(outputDirectory,'multi_planner_animation.mp4');
    try
        video=VideoWriter(animationFile,'MPEG-4');video.FrameRate=comparison.animationVideoFrameRate;open(video);
    catch exception
        warning('Animation:VideoWriter','Khong mo duoc MP4: %s',exception.message);video=[];animationFile='';
    end
end
for frameIndex=1:numel(animationTimes)
    timeNow=animationTimes(frameIndex);
    for i=1:runCount
        if ~valid(i),continue;end
        simulation=runs(i).simulation;
        sampleIndex=find(simulation.time<=timeNow,1,'last');
        if isempty(sampleIndex),sampleIndex=1;end
        pose=[simulation.x(sampleIndex),simulation.y(sampleIndex),simulation.theta(sampleIndex)];
        vertices=transformRobotFootprint(pose,comparisonResult.mapRobot);
        set(trailHandles(i),'XData',simulation.x(1:sampleIndex),'YData',simulation.y(1:sampleIndex));
        set(robotHandles(i),'XData',vertices(:,1),'YData',vertices(:,2));
        nose=pose(1:2)+0.28*[cos(pose(3)) sin(pose(3))];
        set(headingHandles(i),'XData',[pose(1) nose(1)],'YData',[pose(2) nose(2)]);
        status=ternary(timeNow>=simulation.time(end),'DONE',sprintf('t=%.1f s',timeNow));
        title(axesHandles(i),sprintf('%s | %s',strrep(runs(i).planner.name,'_',' '),status), ...
            'FontSize',8);
    end
    drawnow;
    if ~isempty(video)
        try
            writeVideo(video,getframe(figureHandle));
        catch
            video=[];
        end
    end
    if comparison.animationPlaybackSpeed>0
        pause(comparison.animationFrameTime/comparison.animationPlaybackSpeed);
    end
end
if ~isempty(video),close(video);end
exportgraphics(figureHandle,fullfile(outputDirectory,'animation_final_frame.png'),'Resolution',160);
end

function value=lastTime(run)
if isempty(fieldnames(run.simulation)),value=0;else,value=run.simulation.time(end);end
end
function value=ternary(condition,a,b)
if condition,value=a;else,value=b;end
end
