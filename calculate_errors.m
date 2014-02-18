function [e_theta e_ct] = calculate_errors(p,theta, p1, p2)
   
    x1 = p1(1);
    y1 = p1(2);
    x2 = p2(1);
    y2 = p2(2);
    
    %Goal Theta
    theta_g = atan2((y1 - y2), (x1 - x2));
    %Angular error, with care for rollover
    e_theta = atan2(sin(theta_g - theta), cos(theta_g - theta));
    
    % Distance from point to line segment
    % http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    e_ct = det([p2-p1;p-p1])/norm(p2-p1);
end
