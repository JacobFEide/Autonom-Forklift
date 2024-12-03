clc;
clear;
close all;

% Waypoints for roboten å følge
waypoints = [
             2.5, 9.0;
             3.0, 9.0;
             3.5, 9.0;
             4.0, 9.0;
             4.5, 9.0;
             5.0, 9.0;
             5.5, 9.0;
             6.0, 9.0;
             6.5, 8.5;
             6.5, 8.0;
             6.5, 7.5;
             6.5, 7.0;
             6.5, 6.5;
             6.5, 6.0;
             6.5, 5.5;
             6.0, 5.0;
             5.5, 5.0;
             5.0, 5.0;
             4.5, 5.0;
             4.0, 5.0;
             3.5, 5.0;
             3.0, 5.0;
             2.5, 5.0];

% Startpose [x, y, theta]
robotPose = [2.0, 9.0, 0]';

% Målpose
goal = [2.0, 5.0, pi];

% Kjøretøyparametre
L = 0.90; % Akselavstand
v = 0.5; % Konstant hastighet
sampleTime = 0.1;
vizRate = rateControl(10);
goalRadius = 0.2;
distanceToGoal = norm(robotPose(1:2) - goal(1:2));
maxSteeringAngle = pi/4; % Maksimal styrevinkel

% Pure Pursuit kontroller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.DesiredLinearVelocity = v;
controller.LookaheadDistance = 0.5;

% Plot oppsett
figure;
plot(waypoints(:,1), waypoints(:,2), 'k--o');
hold on;
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
title('Pure Pursuit');
xlabel('X [m]');
ylabel('Y [m]');
xlim([-1, 10]);
ylim([-1, 10]);
grid on;

% Simuleringssløyfe
while distanceToGoal > goalRadius
    % Beregn kontrollkommandoer
    [v, omega] = controller(robotPose);
    
    % Beregn styrevinkel \(\delta\) basert på \(\omega\)
    delta = atan2(omega * L, v);
    delta = max(min(delta, maxSteeringAngle), -maxSteeringAngle); % Begrens styrevinkelen

    % Oppdater robotens pose ved hjelp av bicycle model
    x = robotPose(1);
    y = robotPose(2);
    theta = robotPose(3);

    % Kinematiske oppdateringer
    x = x + v * cos(theta) * sampleTime;
    y = y + v * sin(theta) * sampleTime;
    theta = theta + (v / L) * tan(delta) * sampleTime;
    theta = wrapToPi(theta);

    % Oppdater robotens posisjon
    robotPose = [x; y; theta];
    distanceToGoal = norm(robotPose(1:2) - goal(1:2));

    % Plot robotens bane og posisjon
    cla;
    plot(waypoints(:,1), waypoints(:,2), 'k--o');
    plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plotRobot(robotPose, 2, 1);
    plot(robotPose(1), robotPose(2), 'ro');
    drawnow;

    % Vent til neste visualiseringsoppdatering
    waitfor(vizRate);
end

disp('Goal reached!');

% Funksjon for å tegne roboten som et rektangel
function plotRobot(pose, length, width)
    theta = pose(3);
    x = pose(1);
    y = pose(2);

    % Definer hjørnene av roboten
    corners = [
        -length/2, -width/2;
        length/2, -width/2;
        length/2, width/2;
        -length/2, width/2
    ];

    % Roter og transler hjørnene basert på robotens posisjon og orientering
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    corners = (R * corners')' + [x, y];

    % Tegn roboten som et fylt rektangel
    fill(corners(:,1), corners(:,2), 'g', 'FaceAlpha', 0.3);
    hold on;
end
