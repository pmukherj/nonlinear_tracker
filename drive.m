clc;clear all;close all

%Set up simulation parameters
tf = 60;
dt = 0.1;

%Initial Conditions
x0 = 0;
y0 = 0;
theta0 = 0;

%Define path to be followed and related threshold
p = [3 5; 4 -4;-5 5; -4 -3];
%How close do you want to be to each point?
dist_thresh = 0.1;
%Path Controller parameters
k_p = 1;
g = 1; %goal number

%Setup first path segment
p1 = [x0 y0];
p2 = p(g,:);
x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);
plot([x1 x2],[y1 y2], 'r');

%Pre allocated matrices for speed
t = 0:dt:tf;
x = zeros(length(t));
y = zeros(length(t));
theta = zeros(length(t));
d = zeros(length(t),1);
x(1) = x0;
y(1) = y0;
theta(1) = theta0;

%Setup plotting
h = figure(1);
hold on;
axis([-6 6 -6 6]);


for i=2:length(t) 
    %% Plot Goal Line (and segment ends)
    clf;
    hold on; 
    axis([-6 6 -6 6]);
    plot([x0; p(:,1)], [y0; p(:,2)]);
    scatter(p(:,1), p(:,2),'g');
    
    diff = [x(i-1) y(i-1)] - p2;
    dist = norm(diff);
    
    if (dist < dist_thresh)
        if (g==numel(p(:,1)))
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

    plot([x1 x2],[y1 y2], 'g');
    
    %% Plot Perpendicular crosstrack and angles error
    
    m = (y2 - y1)/(x2 - x1);
    b = y1 - m*x1;
    m_p = -1/m;
    b_p = y(i-1) - m_p*(x(i-1));
    %Find the intersection point
    % m*x + b = m_p*x + b_p; 
    % (m - m_p)*x = b_p - b;
    x_int = (b_p - b) / (m-m_p);
    y_int = m*x_int + b;
    plot([x(i-1) x_int], [y(i-1) y_int], 'r');

    %% Calculate errors
    [e_t d(i)] = calculate_errors([x(i-1) y(i-1)], theta(i-1), p1, p2);
    
    %% Calculate Controller
    k_v = 0.1;
    V_max = 1;
    V_min = 0.2;
%   V = 1; %constant forward velocity (not that great of an idea)
    V = k_v/abs(e_t);
    if (V>V_max)
        V = V_max;
    elseif (V<V_min)
        V = V_min;
    end
    w = e_t + atan2(k_p * d(i),V);

    %% Simulate dynamics
    x(i) = x(i-1) + V*cos(theta(i-1))*dt;
    y(i) = y(i-1) + V*sin(theta(i-1))*dt;
    theta(i) = theta(i-1) + w*dt;
    
    %Plot Vehicle Position
    drawcar(x(i),y(i),theta(i),0.5,h); 
    title('Nonlinear path tracker (cross track error marked in red)')
    pause(dt);
    
    
end
%% Plot Some results
figure(2)
plot(t, d);
xlabel('Time(t)');
ylabel('Crosstrack error(m)');

