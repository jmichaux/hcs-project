function dz = lintraj(tdummy,in2,udummy)
%LINTRAJ
%    DZ = LINTRAJ(TDUMMY,IN2,UDUMMY)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    04-Dec-2020 17:10:27

accx = in2(7,:);
accy = in2(8,:);
accth = in2(9,:);
t = in2(10,:);
velx0 = in2(4,:);
vely0 = in2(5,:);
velth0 = in2(6,:);
dz = [velx0+accx.*t;vely0+accy.*t;velth0+accth.*t;0.0;0.0;0.0;0.0;0.0;0.0;1.0];
