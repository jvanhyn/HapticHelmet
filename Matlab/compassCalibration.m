close all
clear
clc

device = serialport("/dev/cu.usbmodem142101",9600);
FUSE = imufilter;
FUSE.OrientationFormat = 'Rotation matrix';

e1 = [1 0 0]';
e2 = [0 1 0]';
e3 = [0 0 1]';

i = 1;

while true
    data = readline(device);
    str = split(data,',');
    accelReadings = double(str(1:3))'*9.81;
    gyroReadings = double(str(4:6))'/180*pi;
    magReadings(:,i) = double(str(7:9))';
    [orientation,angularVelocity] = FUSE(accelReadings,gyroReadings);
    v1 = orientation*e1;

    %if(mod(i,100)==0)
    % 
    % pbaspect([1 1 1])
    % drawnow
    % hold off
    % hold on
    %axis([-20 20 -20 20 -20 20])
    %end
   
    i = i+1;
    
end

% device = serialport("/dev/cu.usbmodem142101",9600);
% 


%;
%[orientation,angularVelocity] = FUSE(accelReadings,gyroReadings)

% q = 0.5;
% r = 0.5;
% s = 0.5;

% alpha = atan2(r,q)

% u = q*cos(-alpha)-r*sin(-alpha);
% v = q*sin(-alpha)+r*cos(-alpha);
% w = s;

% beta = atan2(w,u)

% x = u*cos(-beta)-w*sin(-beta);
% y = v;
% z = u*sin(-beta)+w*cos(-beta);


% q = quiver3(0,0,0,q,r,s)
% q.LineWidth = 2;
% q.ShowArrowHead = 'off';
% hold on
% q = quiver3(0,0,0,x,y,z)
% q.LineWidth = 2;
% q.ShowArrowHead = 'off';