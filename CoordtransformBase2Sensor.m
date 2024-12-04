% Transform for Camera to Base
%Konstanter
%Plasering av kamera
t = [0; 0; 0.25]; 
%vinkel av rotasjon 
tet = pi/2; 
%Rotasjonsmatrise
R = [1, 0, 0; 0, cos(tet), -sin(tet); 0, sin(tet), cos(tet)];

%Utregning av senoor over end effektor
sTee = [R, t; 0, 0, 0, 1]; 
%Definerer base til end effektor fra DH-Parameter scriptene 
bTee = [1, 0, 0, 1.6739; 0, 0, -1, 0; 0, 1, 0, 0.0605; 0, 0, 0, 1];
%Regner ut base til sesnor matrisen
bTs = bTee * sTee;