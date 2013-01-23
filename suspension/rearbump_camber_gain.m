clear all;
close all;
%% This m-script calculates the camber change in bump for the RHS wheel
% centerline in front view under bump
% sign convension: top in = negative camber, anti-clockwise rotation =
% positive rotaion




%% Define suspension geometry points
x1 = 240; y1 = 280; % Upper pickup point A
x2 = 230; y2 = 125; % Lower pickup point B
x3 = 563.5; y3 = 125; % Lower balljoint point C
x4 = 563.5; y4 = 375; % Upper balljoint point D
t = 1200; % track width

ang_step_rad = 0.00015; % this is the step increase of alpha

TD = sqrt((t/2-x4)^2 + (y4-0)^2);
AB = sqrt((x1-x2)^2 + (y1-y2)^2);
%CD = sqrt((x3-x4)^2 + (y3-y4)^2);
AD = sqrt((x1-x4)^2 + (y1-y4)^2);
[alpha,BC] = cart2pol(x3-x2, y3-y2);% find the original position of C relative to B which is a static point
[ang_TD_baseline,TD] = cart2pol(t/2-x4, 0-y4); % find the position of T relative to D
n = 1000; % number of points/iterations

[camber_baseline, CD] = cart2pol(x4-x3,y4-y3); % this gives the starting angle of DC to the verticle with the given sign convension, in radians
%% Main algorithm

for i = 1:n+1;
    alpha_rad(i)= alpha + ang_step_rad * (i-n/2);
    alpha_deg(i)= alpha_rad(i)*180/pi;
    [x3_diff(i),y3_diff(i)] = pol2cart(alpha_rad(i), BC); % now we have the new location of LBJ, point C, relative to B,(x2,y2);
    x3_plot(i)= x2 + x3_diff(i);
    y3_plot(i)= y2 + y3_diff(i);
    AC = sqrt((x1-x3_plot(i))^2+(y1-y3_plot(i))^2);
    ang_ACB(i) = acos((AC^2+BC^2 - AB^2)/(2*AC*BC)); % this finds the angle ACB in radians
    ang_DCA(i) = acos((CD^2+AC^2 - AD^2)/(2*CD*AC)); % this finds the angle DCA in radians
    ang_CD(i) = pi - (ang_DCA(i) + ang_ACB(i) - alpha_rad(i)); % this finds the angle between DC from the horizontal line (as the angle in polar system)
    [x_diff,y_diff] = pol2cart(ang_CD(i), CD); % this gives the relative location of D to C
    x4_plot(i) = x3_plot(i) + x_diff; 
    y4_plot(i) = y3_plot(i) + y_diff; % now the position of the UBJ, point C, is located
    CD_check(i) = sqrt((x4_plot(i)-x3_plot(i))^2 + (y4_plot(i)-y3_plot(i))^2); % CD should be constant since it's on a rigid body
    %[ang_CD(i),length] = cart2pol(x4_plot(i)-x3_plot(i), y4_plot(i)-y3_plot(i)); % this gives the angle of DC to the horizon, length is not used
    
    % camber gain calculation
    Camber_Gain_rad(i) = camber_baseline - ang_CD(i);% this shows the camber gain in radians
    Camber_Gain_deg = Camber_Gain_rad * 180/pi; % converts to degrees
    
    % bump height calculation
    ang_TD(i) = ang_TD_baseline - Camber_Gain_rad(i); % minus sign due to sign convention. now the new angle of TD to the horizontal line is found
    
    [x5_diff(i),y5_diff(i)] = pol2cart(ang_TD(i),TD); % x5_diff and y5_diff gives the location of the contact patch, point T relative to D
    bump_height(i) = y4_plot(i) + y5_diff(i); % eventually the height change of the contact patch, point T, is found
     
end
x5 = x4_plot + x5_diff;
y5 = y4_plot + y5_diff;

%% plot the results

plot(x3_plot, y3_plot,'blue', x4_plot, y4_plot, 'red');
xlabel('x position');
ylabel('y position');
title('Lower and Upper Ball Joints Migration');

figure
plot(x5,y5);
xlabel('x position');
ylabel('y position');
title('Contact Patch Migration');

figure
plot(bump_height, Camber_Gain_deg,'r');
xlabel('bump height / mm');
ylabel('camber gain / deg');
title('Rear Bump Camber Gain');
grid on