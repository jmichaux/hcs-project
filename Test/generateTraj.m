function [xpos, ypos] = generateTraj(x,y,vx,vy,ax,ay)
timestep = 1;
t = linspace(0,1,1000);
vx_initial = vx - ax*timestep;
vy_initial = vy - ay*timestep;
x_initial = x - vx_initial*timestep -(1/2)*ax*timestep^2;
y_initial = y - vy_initial*timestep -(1/2)*ay*timestep^2;
xpos = x_initial + vx_initial * t + (1/2)*ax*t.^2;
ypos = y_initial + vy_initial * t + (1/2)*ay*t.^2;

end