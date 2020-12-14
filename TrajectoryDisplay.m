
clear all
%Initial Condition
x = 8;
y = 10;
theta = 0;
vx = 0;
vy = 0;
vtheta = 0;
scatter( x, y,50,'filled');

%Goal point (waypoints can be inserted)
xgoal = [0];
ygoal = [0];
endcondition = 0.2;

for i = 1:length(xgoal)
    scatter( xgoal(i), ygoal(i),50,'filled');
while (norm([x-xgoal(i),y-ygoal(i)]) > endcondition)
    generateZonotope(x,y,theta,vx,vy,vtheta);
    [xout,yout,thetaout,vxout,vyout,vthetaout,axout,ayout,athetaout] = optimizeAcceleration(xgoal(i),ygoal(i));
    x = xout;
    y = yout;
    theta = thetaout;
    vx =vxout;
    vy = vyout;
    vtheta = 0;
    if i == length(xgoal)
       endcondition = 0.5;
    end
        
end
end
xlabel('x (m)');
ylabel('y (m)');
axis equal



