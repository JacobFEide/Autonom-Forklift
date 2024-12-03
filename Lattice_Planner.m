clc;
clear all;

%Oppretter en matrise med nullere som representerer frie områder 
lager = zeros(100,100);

%Setter cellene i 'lager' til 1 for å markere hindringer
lager(20:40,1:60) = 1;
lager(60:80,1:60) = 1;
lager(1:100,1:2) = 1;
lager(1:2,1:100) = 1;
lager(1:100,99:100) = 1;
lager(99:100,1:100) = 1;


% Opprett Lattice Planner
lp = Lattice(lager, 'grid', 5, 'root', [20 90 0]); 
lp.plan('iterations', 40, 'cost', [1 10 10]);
lp.plot();
% Spørring om rute
startPose = [20, 90, 0];    % Startposisjon [x, y, vinkel]
goalPose = [20, 50, pi];     % Målposisjon [x, y, vinkel]
lp.query(startPose, goalPose);
lp.plot();



