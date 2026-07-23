function animateRobot(map, simulation, config, reference)
%ANIMATEROBOT Hien footprint, huong, mode va vet da di cua robot.
if nargin<4,reference=struct();end
figure('Name','Robot animation','Color','w');
drawOccupancyMap(map); grid on;
if isfield(reference,'x')
    plot(reference.x,reference.y,'-','Color',[0.65 0.65 0.65], ...
        'LineWidth',1.0,'DisplayName','Reference');
end
trajectoryHandle = plot(nan,nan,'b-','LineWidth',1.3);
footprintHandle = patch(nan,nan,[0.1 0.55 1], ...
    'FaceAlpha',0.35,'EdgeColor',[0 0.2 0.7],'LineWidth',1.3);
headingHandle = plot(nan,nan,'r-','LineWidth',2);
titleHandle = title('');
for k = 1:config.animationSkip:numel(simulation.time)
    vertices = transformRobotFootprint([simulation.x(k),simulation.y(k), ...
        simulation.theta(k)],config.robot);
    set(trajectoryHandle,'XData',simulation.x(1:k),'YData',simulation.y(1:k));
    set(footprintHandle,'XData',vertices(:,1),'YData',vertices(:,2));
    nose=[simulation.x(k),simulation.y(k)]+0.28*[cos(simulation.theta(k)), ...
        sin(simulation.theta(k))];
    set(headingHandle,'XData',[simulation.x(k) nose(1)], ...
        'YData',[simulation.y(k) nose(2)]);
    mode='UNKNOWN';
    if isfield(reference,'mode')
        index=max(1,min(numel(reference.mode),round(simulation.referenceIndex(k))));
        mode=reference.mode{index};
        if index==numel(reference.mode) && ...
                abs(simulation.v(k))<config.controller.stationaryThreshold && ...
                abs(simulation.omega(k))>1e-3
            mode='FINAL_ALIGN';
        end
    end
    set(titleHandle,'String',sprintf( ...
        'Adaptive | t=%.2f s | %s | v=%.3f m/s | omega=%.3f rad/s | clearance=%.3f m', ...
        simulation.time(k),mode,simulation.v(k),simulation.omega(k), ...
        simulation.clearance(k)),'Interpreter','none');
    drawnow;
    if config.animationPlaybackSpeed>0
        pause(config.animationSkip*config.dt/config.animationPlaybackSpeed);
    end
end
end
