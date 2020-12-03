function [] = generate_xyth_dynamics()
syms x y th real;
syms accx accy accth t real;
syms velx0 vely0 velth0 real;
syms daccx daccy daccth real;

velx = velx0 + accx*t;
vely = vely0 + accy*t;
velth = velth0 + accth*t;

dt = 1;
% daccx = 0;
% daccy = 0;
% daccth = 0;

z = [x; y; th; velx0; vely0; velth0; accx; accy; accth; t];
dz = [velx; vely; velth; 0; 0; 0; 0; 0; 0; dt];

syms tdummy udummy real;
matlabFunction(dz, 'File', 'lintraj', 'vars', {tdummy, z, udummy});

end