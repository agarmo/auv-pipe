function [C] = C_calc( nu )
%C_CALC computes the coriolis matrix, -uses the nu-vector as input.

m = 1380;
x_g = 0;
y_g = 0;
z_g = 0.0328;
I_x = 86.53;
I_y = 1247;
I_z = 1247;


u = nu(1);
v = nu(2);
w = nu(3);
p = nu(4);
q = nu(5);
r = nu(6);


C = [0 0 0 m*(y_g*q + z_g*r) -m*(x_g*q-w) -m*(x_g*r+v);
     0 0 0 -m*(y_g*p+w) m*(z_g*r+x_g*p) -m*(y_g*r-u);
     0 0 0 -m*(z_g*p-v) -m*(z_g*q+u) m*(x_g*p+y_g*q);
     -m*(y_g*q+z_g*r) m*(y_g*p+w) m*(z_g*p-v) 0 I_z*r -I_y*q;
     m*(x_g*q-w) -m*(z_g*r+x_g*p) m*(z_g*q+u) -I_z*r 0 I_x*p;
     m*(x_g*r+v) m*(y_g*r-u) -m*(x_g*p+y_g*q) I_y*q -I_x*p 0];
     
end
