clear all
x = -1;
y = 4;
theta = 0;
vx = 0;
vy = 0;
vtheta = 0;
xgoal1 = 0;
ygoal1 = 0;
i = 1;
while (norm([x-xgoal1,y-ygoal1]) > 0.1)
    generateZonotope(x,y,theta,vx,vy,vtheta);
    [xout,yout,thetaout,vxout,vyout,vthetaout,axout,ayout,athetaout] = optimizeAcceleration(xgoal1,ygoal1);
    x = xout;
    y = yout;
    theta = thetaout;
    vx =vxout;
    vy = vyout;
    vtheta = 0;
    [xpos,ypos] = generateTraj(x,y,vx,vy,axout,ayout);
    plot(xpos,ypos,'r');
    hold on
end

