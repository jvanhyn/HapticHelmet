close all 
clear
clc

calmagxyz = [8.2400; 7.8950; 8.4100];
device = serialport("/dev/cu.usbmodem142101",9600);
FUSE = imufilter;
FUSE.OrientationFormat = 'Rotation matrix';

i = 1;

f=figure(1)
grid 
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
drawnow
hold on 

while true
    data = readline(device);
    str = split(data,',');
    accelReadings = double(str(1:3))'*9.81;
    gyroReadings = double(str(4:6))'/180*pi;
    calMagReadings = double(str(7:9)) - calmagxyz;
    [orientation,angularVelocity] = FUSE(accelReadings,gyroReadings);
    
    heading(:,i)=calMagReadings\orientajjtion;
    
    
    if(mod(i,10)==0)
    quiver3(0,0,0,heading(1,i),heading(2,i),heading(3,i),'.r',"MarkerSize",10)
    end
    i = i+1;
end



