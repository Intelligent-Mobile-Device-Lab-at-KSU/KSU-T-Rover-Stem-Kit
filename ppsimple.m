clear
clc 
close all

%% Defining Simulation Parameters
drawcustomroute = 0;
corsewaypoints = [1 1; 
    1 2; 
    1 3; 
    1 4; 
    1 5; 
    1 7; 
    1.1 7.5; 
    1.4 8; 
    1.8 8.4; 
    2.5 8.7; 
    4 8.7; 
    6 8.7; 
    8 8.7;
    8.2 8.4;
    8.4 8.4;
    8.6 8;
    8.7 7.5;
    9 7;
    9 5;
    9 4;
    9 3;
    9 2;];

if drawcustomroute
    figure
    axh = axes;
    hold(axh, 'on')
    xy = [0,0];
    corsewaypoints = [];
    xlim([0 10])
    ylim([0 10])
    while ~isempty(xy)
        xy = ginput(1); % click on screen hit enter when done
        if ~isempty(xy)
            plot(axh, xy(1), xy(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            corsewaypoints = [corsewaypoints; [xy(1), xy(2)]];
        end
    end
end

spacing = 0.05; %6 inches
waypoints = mySmoothPoints(corsewaypoints,spacing);

robotGoal = waypoints(end,:).';
pose = [0 0 pi/2]'; % Initial pose (x y theta w.r.t x-axis global)
sampleTime = 0.05;               % Sample time [s]
tVec = 0:sampleTime:20;          % Time array

goalRadius = .5;
distanceToGoal = norm(pose(1:2) - robotGoal);

DesiredLinearVelocity = 5;
MaxTurnAngle = deg2rad(14.45); %degrees
MaxAngularVelocity = pi/8; % radians per second not implemented yet
L = 2; % Look ahead
turns = [];
tVec = 0;
figure
while( distanceToGoal > goalRadius )
    d = L;
    for i=length(waypoints):-1:1
        W = waypoints(i,:);
        lx = W(1,1);
        ly = W(1,2);
        x2 = pose(1);
        y2 = pose(2);
        d = sqrt((lx-x2)^2+(ly-y2)^2);
        if d <= L
            break
        end
    end
    hold off
    plot(waypoints(:,1),waypoints(:,2),"k.-","MarkerSize",8); hold on;
    xlim([0 10])
    ylim([0 10])
    plot(x2,y2,"rx","MarkerSize",10);
    plot(lx,ly,"bx","MarkerSize",10);
    circle(pose(1),pose(2),L);
drawnow
hold off
    theta = pose(3); % car heading relative to world x-axis
    beta = atan2( (ly-pose(2)), (lx-pose(1)) ); % direction in radians to goal point
    if abs(theta-beta)<.000001
        gamma = 0;
    else
        gamma = theta - beta; % direciton in radians to goal point in car's local coordinate where positive is right
    end
    x_offset = d*sin(gamma)*-1;
    y_offset = d*cos(gamma);
    turnangle = (2*x_offset)/(d^2);
    
    thesign  = sign(  (sin(pose(3))*(lx-pose(1))) - (cos(pose(3))*(ly-pose(2))) )
    turnangle = thesign*turnangle;
    % Ensure the turn control saturates at MaxTurnAngle defined by servo
    if abs(turnangle) > MaxTurnAngle
        turnangle = thesign*MaxTurnAngle;
    end
    turnangle = rem(turnangle,2*pi);
    turns = [turns turnangle];
    angularw = turnangle*DesiredLinearVelocity;
    %sign(  (sin(robotCurrentPose(3))*(lx-robotCurrentPose(1))) - (cos(robotCurrentPose(3))*(ly-robotCurrentPose(2))) )
   
    pose(3) = pose(3) + turnangle;
    v =  pose(1:2) - [lx; ly];
    direc = -(v/norm(v)) * DesiredLinearVelocity;
    pose(1:2) = pose(1:2) + direc*sampleTime;
    %pose(2) = pose(2) + DesiredLinearVelocity*sampleTime*cos(turnangle);
    %pose(1) = pose(1) + DesiredLinearVelocity*sampleTime*sin(turnangle);
    distanceToGoal = norm(pose(1:2) - robotGoal);
    tVec = [tVec tVec(end)+sampleTime];
end

figure
plot(tVec(2:end),rad2deg(turns),'o')
ylabel('Turn Command in Degrees')
xlabel('Simulation Time (s)')

function h = circle(x,y,r)
 th = 0:pi/50:2*pi;
 xunit = r * cos(th) + x;
 yunit = r * sin(th) + y;
 h = plot(xunit, yunit,'k');
end

function w = mySmoothPoints(W,spacing)
    w = [];    
    for i=1:length(W)-1
        v = W(i+1,:) - W(i,:);
        d = sqrt((W(i+1,1)-W(i,1))^2+(W(i+1,2)-W(i,2))^2);
        num_points_that_fit = ceil(d/spacing);
        vd = (v/norm(v)) * spacing;
        for k=0:num_points_that_fit-1
            w = [w; (W(i,:) + (vd*k))];
        end
%         plot(W(:,1),W(:,2),"kx-"); hold on;
%         plot(w(:,1),w(:,2),"ro"); hold on;
%         xlim([0 10])
%         ylim([0 10])
%         hold off
    end
    w = [w; W(end,:)];
end