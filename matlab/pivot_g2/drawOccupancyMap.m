function drawOccupancyMap(map)
%DRAWOCCUPANCYMAP Ve occupancy grid voi truc toa do met.
imagesc([0 map.width],[0 map.height],double(map.occupancy));
set(gca,'YDir','normal');
axis equal tight;
colormap(gca,[1 1 1;0.18 0.18 0.18]);
xlabel('x (m)'); ylabel('y (m)');
hold on;
end
