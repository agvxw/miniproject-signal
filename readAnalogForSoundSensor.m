clc;
clear all;

a = arduino('com3','uno');
v = readVoltage(a,'A2');
s = (v*1023.0)/5 ;

%record and plot 10s
i = 0;

x=500;
tic
while toc < 30
    i = i+1;
    v = readVoltage(a,'A2');
    s = (v*1023.0)/5 ;
    x = [x,s];
    plot(x);
    grid ON ;
        
    t(i)= toc;
    drawnow 
end


