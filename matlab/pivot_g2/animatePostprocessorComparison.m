function animationFile = animatePostprocessorComparison(result,outputDirectory,comparison)
%ANIMATEPOSTPROCESSORCOMPARISON Sau robot tren cung cua so/cung thoi gian.
runs=result.runs;map=result.map;count=numel(runs);colors=lines(count);
valid=arrayfun(@(r)~isempty(fieldnames(r.simulation)),runs);
figureHandle=figure('Name','Synchronized postprocessor animation', ...
    'Color','w','Position',[20 35 1600 920]);
layout=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
axesHandles=gobjects(count,1);trailHandles=gobjects(count,1);
robotHandles=gobjects(count,1);headingHandles=gobjects(count,1);
for i=1:count
    axesHandles(i)=nexttile(layout);axes(axesHandles(i)); %#ok<LAXES>
    drawOccupancyMap(map);grid on;
    if valid(i)
        plot(runs(i).reference.x,runs(i).reference.y,'--', ...
            'Color',[0.55 0.55 0.55],'LineWidth',0.9);
    end
    trailHandles(i)=plot(nan,nan,'-','Color',colors(i,:),'LineWidth',1.5);
    robotHandles(i)=patch(nan,nan,colors(i,:),'FaceAlpha',0.38, ...
        'EdgeColor',colors(i,:),'LineWidth',1.2);
    headingHandles(i)=plot(nan,nan,'r-','LineWidth',1.3);
    title(runs(i).method.displayName,'Interpreter','none','FontSize',9);
end
sgtitle(layout,sprintf('%s - %s/%s | same physical time', ...
    result.planner.name,map.name,result.scenario.name),'Interpreter','none');
maximumTime=max(arrayfun(@lastTime,runs));
times=(0:comparison.animationFrameTime:maximumTime).';
if isempty(times),times=0;elseif times(end)<maximumTime,times(end+1)=maximumTime;end
animationFile='';video=[];
if comparison.saveAnimation
    animationFile=fullfile(outputDirectory,'postprocessor_animation.mp4');
    try
        video=VideoWriter(animationFile,'MPEG-4');
        video.FrameRate=comparison.animationVideoFrameRate;open(video);
    catch exception
        warning('Animation:VideoWriter','Khong tao duoc MP4: %s',exception.message);
        video=[];animationFile='';
    end
end
for frameIndex=1:numel(times)
    timeNow=times(frameIndex);
    for i=1:count
        if ~valid(i),continue;end
        simulation=runs(i).simulation;
        sample=find(simulation.time<=timeNow,1,'last');
        if isempty(sample),sample=1;end
        pose=[simulation.x(sample),simulation.y(sample),simulation.theta(sample)];
        vertices=transformRobotFootprint(pose,result.robotConfig);
        set(trailHandles(i),'XData',simulation.x(1:sample), ...
            'YData',simulation.y(1:sample));
        set(robotHandles(i),'XData',vertices(:,1),'YData',vertices(:,2));
        nose=pose(1:2)+0.28*[cos(pose(3)) sin(pose(3))];
        set(headingHandles(i),'XData',[pose(1) nose(1)], ...
            'YData',[pose(2) nose(2)]);
        if timeNow>=simulation.time(end),status='DONE';
        else,status=sprintf('t = %.1f s',timeNow);end
        title(axesHandles(i),sprintf('%s | %s', ...
            runs(i).method.displayName,status),'Interpreter','none','FontSize',8);
    end
    drawnow;
    if ~isempty(video)
        try,writeVideo(video,getframe(figureHandle));
        catch exception
            warning('Animation:Frame','Dung ghi video: %s',exception.message);
            try,close(video);catch,end
            video=[];animationFile='';
        end
    end
    if comparison.animationPlaybackSpeed>0
        pause(comparison.animationFrameTime/comparison.animationPlaybackSpeed);
    end
end
if ~isempty(video),close(video);end
exportgraphics(figureHandle,fullfile(outputDirectory, ...
    'animation_final_frame.png'),'Resolution',170);
end

function value=lastTime(run)
if isempty(fieldnames(run.simulation)),value=0;
else,value=run.simulation.time(end);end
end
