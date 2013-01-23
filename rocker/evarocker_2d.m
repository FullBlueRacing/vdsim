%% This function evaluates the installation ratio for a 2D pushrod
% suspension geometry setup. 2D means that the rocker-damper-pushrod
% mechanism is always in the same plane
%
% syntax:
%
% [bump_height IR] = evarocker_2d(L1,L2,angle_rocker,Ls0)
%
% where:
%Ls0: static shock length
%L1: mm, shock side rokcer arm length
%L2: mm, upright side rocker arm length
%angle_rocker: degrees, angle between rocker arms
%
% Author: Yunlong Xu, Full Blue Racing
function [bump_height IR] = evarocker_2d(L1,L2,angle_rocker,Ls0)
    if nargin < 4;
        Ls0 = 176;end
    if nargin < 3;
        Ls0 = 176;
        angle_rocker = 90;end
    
    
    %% Define suspension geometry points
    x1 = 240; y1 = 290; % Upper pickup point A
    x2 = 230; y2 = 125; % Lower pickup point B
    x3 = 563.5; y3 = 125; % Lower balljoint point C
    x4 = 563.5; y4 = 375; % Upper balljoint point D
    t = 1200; % track width
    
    
    TD = sqrt((t/2-x4)^2 + (y4-0)^2);
    AB = sqrt((x1-x2)^2 + (y1-y2)^2);
    %CD = sqrt((x3-x4)^2 + (y3-y4)^2);
    AD = sqrt((x1-x4)^2 + (y1-y4)^2);
    [alpha,BC] = cart2pol(x3-x2, y3-y2);% find the original position of C relative to B which is a static point
    [ang_TD_baseline,TD] = cart2pol(t/2-x4, 0-y4); % find the position of T relative to D
    n = 76; % number of points/iterations 76
    
    [camber_baseline, CD] = cart2pol(x4-x3,y4-y3); % this gives the starting angle of DC to the verticle with the given sign convension, in radians
    
    %% Define Rocker & Damper Geometry and locations
    x6 = 200; % rocker mounting point
    y6 = 380;
    d2 = 20; % damper mounting bracket offset, mm
    d2_ang = 0; % Rocker mounting braket orientation with horizon, degrees
    v2 = 97.81; % damping mounting point node vertical offset
    x7 = 15; % damper mounting point
    y7 = 426.5;
    
    Ls0 = 175; % static shock length
    L1 = 150; % mm, shock side rokcer arm length
    L2 = 150; % mm, upright side rocker arm length
    angle_rocker = 90; % degrees, angle between rocker arms
    angR = angle_rocker*pi/180; % convert angle to radius
    
    %now find pushrod length P
    
    gamma0 = atan((y7-y6)/(x6-x7)); % angle of M to horizon (M is the line between rocker mount and damper mount)
    M = sqrt((y7-y6)^2 + (x7-x6)^2);
    theta0 = acos((L1^2 + M^2 - Ls0^2)/(2*L1*M)); % find the angle between M and L1
    delta0 = pi-gamma0-theta0-angR; % delta is the polar angle of a0 b0 relative to x6 y6
    [a0diff, b0diff] = pol2cart(delta0, L2);
    Pab = [x6,y6]+ [a0diff, b0diff] % find the initial location of rocker/pushrod connection point
    a0 = Pab(1);
    b0 = Pab(2);
    P = sqrt((a0-x3)^2 + (b0 - y3)^2) % The length of pushrod
    
    
    % test if the geometry is valid
    
    if Ls0 + L1 <= sqrt((x7-x6)^2 + (y7-y6)^2);
        
        %display('invalid geometry!, L1 too short!')
        IR = 1:n;
        bump_height = 1:n;
        %     IR = [1:100:(1+100*(n-1))];  % creat IR for format consistency, this IR must be bad enough to be automatically picked up by batch and throw away
        IR = transpose(IR);
        %     bump_height = [1:100:(1+100*(n-1))];
        bump_height = transpose(bump_height);
    else
        
        
        
        
        
        
        %% Main algorithm
        
        %set zero vecotrs to avoid growing an array inside a for loop to increase
        %speed!
        
        
        
        
        
        %bump calculation as before
        alpha_rad = alpha + [-(n-1)/2*0.002 : 0.002: 0.002*((n-1)/2)];
        alpha_deg = alpha_rad*180/pi;
        [x3_diff,y3_diff] = pol2cart(alpha_rad, BC); % now we have the new location of LBJ, point C, relative to B,(x2,y2);
        x3_plot= x2 + x3_diff; %new LBJ coordinates
        y3_plot= y2 + y3_diff;
        AC = sqrt((x1-x3_plot).^2+(y1-y3_plot).^2);
        ang_ACB = acos((AC.^2+BC^2 - AB^2)/(2*AC*BC)); % this finds the angle ACB in radians
        ang_DCA = acos((CD^2+AC.^2 - AD^2)/(2*CD*AC)); % this finds the angle DCA in radians
        ang_CD = pi - (ang_DCA + ang_ACB - alpha_rad); % this finds the angle between DC from the horizontal line (as the angle in polar system)
        [x_diff,y_diff] = pol2cart(ang_CD, CD); % this gives the relative location of D to C
        x4_plot = x3_plot + x_diff;
        y4_plot = y3_plot + y_diff; % now the position of the UBJ, point C, is located
        %CD_check = sqrt((x4_plot-x3_plot)^2 + (y4_plot-y3_plot)^2); % CD should be constant since it's on a rigid body
        %[ang_CD(i),length] = cart2pol(x4_plot(i)-x3_plot(i), y4_plot(i)-y3_plot(i)); % this gives the angle of DC to the horizon, length is not used
        
        % camber gain calculation
        Camber_Gain_rad = camber_baseline - ang_CD;% this shows the camber gain in radians
        
        % bump height calculation
        ang_TD = ang_TD_baseline - Camber_Gain_rad; % minus sign due to sign convention. now the new angle of TD to the horizontal line is found
        
        [x5_diff,y5_diff] = pol2cart(ang_TD,TD); % x5_diff and y5_diff gives the location of the contact patch, point T relative to D
        bump_height = y4_plot + y5_diff; % eventually the height change of the contact patch, point T, is found
        bump_height = transpose(bump_height);
        
        
        %         x5 = x4_plot + x5_diff;
        %         y5 = y4_plot + y5_diff;
        %         Camber_Gain_deg = Camber_Gain_rad * 180/pi; % converts to degrees
        
        %% now calculate the motion ratio and etc
        
        % find length of Q (point C to E), rocker mount to LBJ
        Q = sqrt((x3_plot-x6).^2 +(y3_plot - y6).^2);
        % find orientation of Q
        zeta = pi - atan((y6-y3_plot)./(x3_plot-x6));
        % find orientation of P
        beta = acos((Q.^2 + P^2 - L2^2)./(2.*Q*P));
        
        if ~isreal(zeta-beta) % if this is imaginary, then Q-P>L2, not valid geometry for triangle
            IR = 1:n;
        else
            % find locus of (a,b)
            [adiff bdiff] = pol2cart(zeta - beta, P);
            a = x3_plot + adiff;
            b = y3_plot + bdiff;
            
            % find delta, the orientation of L2
            [delta,R] = cart2pol(a-x6, b-y6);
            
            % find angle between M and L1
            theta = pi - angR - delta - gamma0;
            % find new shock length!
            Ls = sqrt(M^2+L1^2 - 2*L1*M*cos(theta));
            
            % Calculate motion ratio
            
            IR = zeros(n,1);
            
            for k = 1:n-1; % so IR will have one point less than other graphs
                IR(k) = (Ls(k)-Ls(k+1))/(bump_height(k+1)-bump_height(k));
                
            end
            
            IR(n) = IR(n-1)+ (IR(n-1)-IR(n-2)); % simply approximate the last point by intepretation to equalise the size of the matrix
            
            
        end
    end
    
    %% plot the results
    
    %    plot(x3_plot, y3_plot,'blue', x4_plot, y4_plot, 'red');
    %     grid on;
    %     xlabel('x position');
    %     ylabel('y position');
    %     title('Lower and Upper Ball Joints Migration');
    %
    %     figure
    %     plot(x5,y5);
    %     grid on;
    %     xlabel('x position');
    %     ylabel('y position');
    %     title('Contact Patch Migration');
    %
    %     figure
    %     plot(bump_height, Camber_Gain_deg);
    %     grid on;
    %     xlabel('bump height / mm');
    %     ylabel('camber gain / deg');
    %     title('Bump Camber Gain');
    
    % figure;
    % scatter_plot_x = [x1 x2 x3 x4 x6 x7];
    % scatter_plot_y = [y1 y2 y3 y4 y6 y7];
    % scatter (scatter_plot_x,scatter_plot_y,200, 'red', 'filled');
    % grid on;
    % title('Suspension Points');
    if nargout <2
        bump_height = IR; % when only one output is called, the output deliverd will be IR
    end
    
%    ang_L2 = delta*180/pi;
plot(bump_height,IR)
grid on
   xlabel('bump/mm')
    ylabel('installation ratio')
end
