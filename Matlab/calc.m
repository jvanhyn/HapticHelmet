% calmagxyz = (max(magReadings')'+min(magReadings')')/2
% plot3(magReadings(1,:),magReadings(2,:),magReadings(3,:),'.');
% pbaspect([1 1 1]);
% hold on
% plot3(calmagxyz(1),calmagxyz(2),calmagxyz(3),'.r',"MarkerSize",10)
% xL = xlim;
% yL = ylim;
% zL = zlim;
% line([0 0], yL,[0 0]);  %x-axis
% line(xL, [0 0],[0 0]);  %y-axis
% line([0 0], [0 0],zL);  %y-axis
% hold off
calMagReadings = magReadings - calmagxyz;
plot3(calMagReadings(1,:),calMagReadings(2,:),calMagReadings(3,:),'.r',"MarkerSize",10)
pbaspect([1 1 1]);
xL = xlim;
yL = ylim;
zL = zlim;
line([0 0], yL,[0 0]);  %x-axis
line(xL, [0 0],[0 0]);  %y-axis
line([0 0], [0 0],zL);  %y-axis
%save('caldata',"magReadings")