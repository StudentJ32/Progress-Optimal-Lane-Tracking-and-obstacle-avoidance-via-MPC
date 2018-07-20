clc
clear

p = 0:0.01:6*pi;
x = 5*cos(p);
y = 5*sin(p);

pp_x = spline(p,x);
pp_y = spline(p,y);
p_dx = fnder(pp_x,x);
p_dy = fnder(pp_y,y);
theta = 0:0.01:6*pi;

plot(ppval(pp_x,theta),ppval(pp_y,theta))

