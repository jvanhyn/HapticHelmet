plot3(heading(1,:),heading(2,:),heading(3,:),'.r',"MarkerSize",10)
pbaspect([1 1 1]);
xlim([-0.2 0.2])
ylim([-0.2 0.2])
zlim([-0.2 0.2])
xL = xlim;
yL = ylim;
zL = zlim;
line([0 0], yL,[0 0]);  %x-axis
line(xL, [0 0],[0 0]);  %y-axis
line([0 0], [0 0],zL);  %y-axis