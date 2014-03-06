clc;clear all;close all
tf = 40;
dt = 0.1;
x0 = 0;
y0 = 0;
theta0 = 0;
dist_thresh = 0.1;

display_on = 1;
p = [3 5; 3 -5;-3 5];
t = 0:dt:tf;

x = zeros(length(t));
y = zeros(length(t));
theta = zeros(length(t));
d = zeros(length(t));

x(1) = x0;
y(1) = y0;
theta(1) = theta0;

k_p = 0.5;
h = figure(1);
hold on;
axis([-6 6 -6 6]);
g = 1; %goal number

p1 = [x0 y0];
p2 = p(g,:);

x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);

plot([x1 x2],[y1 y2], 'r');



for i=2:length(t) 
    %% Plot Goal Line (and segment ends)
    clf;
    hold on; 
    axis([-6 6 -6 6]);

    diff = [x(i-1) y(i-1)] - p2;
    dist = norm(diff);
    
    if (dist < dist_thresh)
        if (g==numel(p))
           break;
        end
        p1 = p(g,:);
        p2 = p(g+1,:);
        g = g + 1;
       
      
    end

     x1 = p1(1);
    y1 = p1(2);
    x2 = p2(1);
    y2 = p2(2);

        plot([x1 x2],[y1 y2], 'r');
    
    %% Plot Perpendicular crosstrack and angles error
    
    m = (y2 - y1)/(x2 - x1);
    b = y1 - m*x1;
    x_line = x1:0.1:x2;
    y_line = m*x_line + b;
    plot(x_line, y_line, 'r');
    m_p = -1/m;
    b_p = y(i-1) - m_p*(x(i-1));
    %Find the intersection point
%     m*x + b = m_p*x + b_p; 
%     (m - m_p)*x = b_p - b;
    x_int = (b_p - b) / (m-m_p);
    y_int = m*x_int + b;
    plot([x(i-1) x_int], [y(i-1) y_int], 'r');

    %% Calculate errors
    [e_t d(i)] = calculate_errors([x(i-1) y(i-1)], theta(i-1), p1, p2);
    
    %% Calculate Controller
    V = 0.5;
    w = e_t + atan2(k_p * d(i),V);

    %% Simulate dynamics
    x(i) = x(i-1) + V*cos(theta(i-1))*dt;
    y(i) = y(i-1) + V*sin(theta(i-1))*dt;
    theta(i) = theta(i-1) + w*dt;
    drawcar(x(i),y(i),theta(i),0.5,h); 
    pause(dt);
    
    
end
%%
figure(2)
plot(t, d);


