
device = serialport("/dev/cu.usbmodem142101",9600);
i = 1;

r=1;
    teta=-pi:0.01:pi;
    x=r*cos(teta);
    y=r*sin(teta);

while true
    data = readline(device);
    str = split(data,',');
    accelReadings = double(str(1:3))'*9.81;
    gyroReadings = double(str(4:6))'/180*pi;
    magReadings(:,i) = double(str(7:9))';
    calmagReadings(:,i) = magReadings(:,i) - calmagxyz;
    

    if(mod(i,100)==0)
    h= quiver(0,0,calmagReadings(1,i),calmagReadings(2,i));
    h.AutoScale = "off";
    xlim([-10 10])
    ylim([-10 10])
    end
    i = i+1;
end