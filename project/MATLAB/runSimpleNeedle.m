% Simple needle model
% Xingchen Fan
close all; clc; clear all;

%% Tisse setup
% tissue properties
F_mean = 40; % nominal reaction force [N]
F_sigma = 0; % reaction force variation due to stiffness variation [N]

% tissue region
H = figure;
tissue = rectangle('Position',[0 0 100 100]);
xlabel('x [mm]')
ylabel('y [mm]')
set(tissue,'LineWidth',2)
axis equal
axis([-20 120 -20 120])
grid on
hold on

% target
target_center = [100 25];
target_radius = 10;
target = rectangle('Position',[target_center-target_radius,...
    2*[target_radius target_radius]],'Curvature',[1 1],...
    'FaceColor','g','EdgeColor','None','LineWidth',5);

% obstacle
obstacle_center = [45 55];
obstacle_radius = 15;
obstacle = rectangle('Position',[obstacle_center-obstacle_radius,...
    2*[obstacle_radius obstacle_radius]],'Curvature',[1 1],...
    'FaceColor','r','EdgeColor','None','LineWidth',5);

%% Needle properties
P_needle.L = 200; % length [mm]
P_needle.r1 = 0.6; % inner diameter [mm]
P_needle.r2 = 0.91; % outer diameter [mm]
P_needle.I = pi/4*((P_needle.r2/1000)^4-(P_needle.r1/1000)^4); % [m^4]
P_needle.E = 5e9; % Young's modulus [Pa]

%% Start inserting

% starting configuration
p0 = [80 100]; % location [mm]
theta0 = -2*pi/3; % agnle [rad]
plot(p0(1),p0(2),'o','LineWidth',2,'MarkerSize',5)
quiver(p0(1)-20*cos(theta0),p0(2)-20*sin(theta0),...
    20*cos(theta0),20*sin(theta0),'LineWidth',2,'MaxHeadSize',50)

%% Keep inserting
v = 2; % insertion step size [mm]
d = 0.4*P_needle.L/v; % number of iterations

R = zeros(2,2,d+1); % rotation matrices
R(:,:,1) = [cos(theta0) -sin(theta0); sin(theta0) cos(theta0)];
theta = theta0;
p = p0;
for i = 1:d
    % random force on the bevel tip, due to uncertain tissue properties
    F = F_mean + F_sigma*randn;
    
    % rotate needle halfway
%     if i <= d/3 
%         dir = 1;
%     elseif i > d/3 && i <= 2*d/3
%         dir = -1;
%     else
%         dir = 1;
%     end
    dir = 1;
    
    [w,theta] = SimpleNeedle(v,F,dir,P_needle);
    
    R(:,:,i+1) = [cos(theta) -sin(theta); sin(theta) cos(theta)]*R(:,:,i);

    grad = R(:,:,i)*[v; w];
    dx = grad(1);
    dy = grad(2);
    
    p = [p; p(i,1)+dx, p(i,2)+dy];
    plot(p(i+1,1),p(i+1,2),'o','LineWidth',2,'MarkerSize',5)
end
plot(p(:,1),p(:,2),'LineWidth',2)

%%
saveas(H,'simulation2.png')