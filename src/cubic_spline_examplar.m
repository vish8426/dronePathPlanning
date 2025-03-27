clear;
clc;

%% Example of Cubic Spline Interpolation on a 2D Space/Plane

x = 0:.25:1;

Y = [sin(x); cos(x)];
xx = 0:.1:1;
YY = spline(x,Y,xx);


plot(x,Y(1,:),'o',xx,YY(1,:),'-')
hold on