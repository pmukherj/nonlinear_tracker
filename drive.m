clc;clear all;close all
tf = 10;
dt = 0.1;
x0 = 0;
y0 = 0;
theta0 = 0;
 
% xg = 3;
% yg = 5;

p1 = [3 5];
p2 = [2 2];

t = 0:dt:tf;

x = zeros(length(t));
y = zeros(length(t));
theta = zeros(length(t));
d = zeros(length(t));

x(1) = x0;
y(1) = y0;
theta(1) = theta0;

k_p = 1;
h = figure(1);
hold on;
axis([-6 6 -6 6]);


for i=2:length(t) 
    %% Plot Goal Line (and segment ends)
    x1 = p1(1);
    y1 = p1(2);
    x2 = p2(1);
    y2 = p2(2);
    m = (y2 - y1)/(x2 - x1);
    b = y1 - m*x1;
    x_line = 0:0.1:10;
    y_line = m*x_line + b;
    plot(x_line, y_line, 'r');
    plot(x1, y1, 'r.');
    plot(x2, y2, 'r.');
    
    %% Plot Perpendicular crosstrack and angles error
    m_p = -1/m;
    b_p = y(i-1) - m_p*(x(i-1));
    
    %Find the intersection point
    %m*x + b = m_p*x + b_p; 
    %(m - m_p)*x = b_p - b;
%     x_int = (b_p - b) / (m-m_p);
%     y_int = m*x_int + b;
%     plot([x(i-1) x_int], [y(i-1) y_int], 'r');
%     plot([x(i-1) x(i-1) + 3*cos(theta(i-1))], [y(i-1) y(i-1) + 3*sin(theta(i-1))],'g');
%     plot([x(i-1) x(i-1) + 3*cos(theta_g)], [y(i-1) y(i-1) + 3*sin(theta_g)],'b');
    
    %% Calculate errors
    [e_t d(i)] = calculate_errors([x(i-1) y(i-1)], theta(i-1), p1, p2);
    
    %% Calculate Controller
    V = 1;
    w = e_t + atan2(k_p * d(i), V);
    
    %% Simulate dynamics
    x(i) = x(i-1) + V*cos(theta(i-1))*dt;
    y(i) = y(i-1) + V*sin(theta(i-1))*dt;
    theta(i) = theta(i-1) + w*dt;
    drawcar(x(i),y(i),theta(i),0.5,h); 
    
    
end
