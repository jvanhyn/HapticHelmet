close all
clear 
clc

device = serialport("/dev/cu.usbmodem142201",9600);

z = [0;0;1];
n = 1;
i = n+1;
m = 10;

while true
    data = readline(device);
    str = split(data,',');
    R(:,i) = double(str(1))';
    z = double(str(2));
    %R(:,i) = 1/(n+1)*(sum(R((i-n):i)'));
    
    
    if(mod(i,m)==0)
    disp(R(:,i)/pi*180)
    theta = 45*z+180;
    A = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
    V1 = A*[cos(pi/8);sin(pi/8)];
    V2 = A*[cos(pi/8);-sin(pi/8)];
    
    plot(cos(0:0.1:2*pi),sin(0:0.1:2*pi));
    hold on
    fill([0 V1(1),V2(1)],[0 V1(2),V2(2)],'b')
    quiver(0,0,cos(R(:,i)),sin(R(:,i)));
    pbaspect([1,1,1])
    xlim([-1,1])
    ylim([-1,1])
    drawnow
    hold off
    end
    i = i+1;
end




