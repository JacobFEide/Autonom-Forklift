% Transform for LiDAR to Base
%Konstanter
%Plasering av kamera
tb = [-0.225; 0; -0.15]; 
tf = [1.775; 0; -0.15];
%vinkel av rotasjon 
tetxb = pi; 
tetyb = (3*pi)/2;
tetyf = pi/2;

%Rotasjonsmatrise bak
Rp = [1, 0, 0; 0, cos(tetyb), -sin(tetyb); 0, sin(tetyb), cos(tetyb) ];
Rr = [cos(tetxb), 0, sin(tetxb); 0, 1, 0; -sin(tetxb), 0, cos(tetxb)];
Rb = Rp*Rr;

%Rotasjonsmatrise foran
Rf = [1, 0, 0; 0, cos(tetyf), -sin(tetyf); 0, sin(tetyf), cos(tetyf) ];

%Utregning av LiDAR til base 
LiBak = [Rb, tb; 0, 0, 0, 1]; 

LiFram = [Rf, tf; 0, 0, 0, 1];
