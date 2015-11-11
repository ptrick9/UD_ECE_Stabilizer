%accelerometer reader
s1 = serial('COM5', 'Baudrate', 250000);
fopen(s1);
str = '';
accX=0;accY=0;accZ=0;
j = 1;
x = 0;
x = 3;
window = 500;
while(1)
    
    sen=str2num(fscanf(s1));
    disp(sen);
    accX(j) = sen(1);
    accY(j) = sen(2);
    accZ(j) = sen(3);
    x(j) = j;
    
    if(j > window)
        x1=x(j-window:j);
        accX1=accX(j-500:j);
        accY1=accY(j-window:j);
        accZ1=accZ(j-window:j);
        xmin=j-window;
        xmax=j;
    else
        x1=x;
        accX1=accX;
        accY1=accY;
        accZ1=accZ;
        xmin=0;
        xmax=window;
    end;
    
    a = [range(accX1) range(accY1) range(accZ1)];
    s = {num2str(a(1)), num2str(a(2)), num2str(a(3))};
    xt = [j-100 j-100 j-100];
    yt = [-1.5 -.5 .5];
    plot(x1,accX1,x1,accY1,x1,accZ1);
    text(xt,yt,s)
    axis([xmin xmax -2 2]);
    drawnow;
    j=j+1;
end;