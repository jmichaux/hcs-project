function [] = generate_xyth_dynamics()

syms velx vely velth accx accy accth t real;
syms velx0 vely0 velth0 real;
syms daccx daccy daccth real;

velx = velx0 + accx*t;
vely = vely0 + accy*t;
velth = velth0 + accth*t;

dt = 1;
daccx = 0;
daccy = 0;
daccth = 0;

x = [velx; vely; velth; accx; accy; accth; t];
dx = [accx; accy; accth; daccx; daccy; daccth; dt];

x = [velx; vely; velth; t];
dx = [accx; accy; accth; dt];

syms tdummy udummy real;
matlabFunction(dx, 'File', 'lintraj', 'vars', {tdummy, x, udummy});

end