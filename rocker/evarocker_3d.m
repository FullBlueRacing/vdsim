%% This function calculates the installation ratio change during suspension
% travel for a 3D pull rod setup. 3-D means that the pull rod -
% damper-rocker mechanism does not always lie in the same plane.  However
% this function does have the capability to work out 2D geometry as well
% with essentially the same input. 
%
% The suspension pickup points need to be specifed in the script
%
% The function can be called using the syntax:
%
% [bump, IR] = evarocker_3d(L1,L2,ang,Ls0)
%
% where
    % L1 is the damper side rocker arm length
    % L2 is the pushrod side rocker arm length
    % ang is the angle between the two arms in degrees
    % LS0 is the static shock eye to eye length
%    
% This function uses extensively another self written function called
% tetra2.m which finds the coordinate of intersection of three spheres. 
%(ie, with 3 known points in the space and their distance to a certain 4th point 
% known as well, find the 4th point)
% this is because this problem occurs very frequently in the problems of
% 3D multibody simulation
%    
% for more information on tetra2.m , try 'help tetra2'
%
% Author: Yunlong Xu, Full Blue Racing




function [bump, IR] = evarocker_3d(L1,L2,ang,Ls0)
    
    % L1 is the damper side rocker arm length
    % L2 is the pushrod side rocker arm length
    % ang is the angle between the two arms in degrees
    % LS0 is the static shock eye to eye length
    ang = ang*pi/180;
    
    
    
    
    %% 12/03/2010
    % This is the basic scrip to simulate the rear suspension travel in bump.
    % This file will serve as the basis for further modifications to make it
    % into a rocker installation ratio evaluation function
    
    % Define Suspension geometry, units mm
    
    % using simpack coordinates 12/03/2010 x: longitudinal, y: sideway, z:
    % vertical origin at the mid-poin of the front wheel centre axis
    
    
    % 13/03/2010
    
    
    
    
    %Top wishbone front pickup, point A
    x01 = 255;
    y01 = 275;
    z01 = 175;
    A = [x01;y01;z01]; % position vector A0
    
    % Top wishebone rear pickup, point B
    x02 = 255;
    y02 = 275;
    z02 = -320;
    B = [x02;y02;z02]; % position vector B0
    
    % Top spherical bearing on upright, point C
    x03 = 580;
    y03 = 362;
    z03 = -5;
    C0 = [x03;y03;z03]; % position vector C0
    
    % Bottom wishbone front pickup, point D
    x04 = 205;
    y04 = 118;
    z04 = 175;
    D = [x04;y04;z04]; % position vector D0
    
    % Bottom wishebone rear pickup, point E
    x05 = 205;
    y05 = 118;
    z05 = -320;
    E = [x05;y05;z05]; % position vector E0
    
    % Bottom spherical bearing on upright, point F
    x06 = 590;
    y06 = 138;
    z06 = 5;
    F0 = [x06;y06;z06]; % position vector F0
    
    % tyre concact patch, point T
    x0t = 625;
    y0t = 0;
    z0t = 0;
    T0 = [x0t;y0t;z0t]; % position vector T0
    
    
    % set track rod coordinates to be lower wishbone end coordinates for
    % front suspension
    % track rod chassis pickup point
    x07 = 170;
    y07 = 138;
    z07 = 15;
    G = [x07;y07;z07]; % position vector G
    
    % track rod uprigth pickup point
    x08 = 590;
    y08 = 138;
    z08 = 15;
    H0 = [x08;y08;z08]; % position vector H0
    
    % rocker mounting hole location
    x09 = 210;
    y09 = 140;
    z09 = -70;
    I = [x09;y09;z09];
    
    % damper mouting hole location
    x10 = 210;
    y10 = 140;
    z10 = -290;
    J = [x10;y10;z10];
    
    
    %scatter3([x01,x02,x03,x04,x05,x06,x0t],[y01,y02,y03,y04,y05,y06,y0t],[z01,z02,z03,z04,z05,z06,z0t])
    
    
     %% Calculate the length of the rigid links and some initial geometry
    
    AB = norm(A-B); % distance between upper pickup points
    DE = norm(D-E); % distance between lower pickup points
    AC = norm(A-C0); % length of the upper front arm
    BC = norm(B-C0); % length of the upper rear arm
    DF = norm(F0-D); % length of the lower front arm
    EF = norm(E-F0); % length of the lower rear arm
    CF = norm(C0-F0); % distance between the spherical joints on the upright
    CT = norm(T0-C0); % distance between the contact patch and the upper sphericaljoint
    CH = norm(H0-C0); % distance between the upper spherical joint and the trackrod pickup point on the upright
    FH = norm(F0-H0); % distance between the lower sphercial joint and the trackrod pickup point on the upright
    GH = norm(G-H0); % length of the track rod
    CT = norm(T0-C0); % length between the upper spherical joint and the tyre contact patch
    FT = norm(T0-F0); % length between the lower spherical joint and the tyre contact patch
    HT = norm(T0-H0); % length between the trackrod pickup point on the upright to the tyre contact patch
    IJ = norm(I-J); % distance between the rocker mount and the damper mount
    IF0 = norm(I-F0); % distance between the rocker mount and the initial LBJ point
    JF0 = norm(J-F0); % distance between the damper mount and the initial LBJ point
    IC0 = norm(I-C0);
    JC0 = norm(J-C0);
    
    % The rocker is restrained to be in the same plane as the IJF0 plane
    axis = cross((J-I),(C0-I))/norm(cross((J-I),(C0-I))) ;% axis of rotation of rocker, positive outwards from the car
    %pause
    
    
    
    %% Main Algorithm
    
    % evaluate the initial geometry
    
    beta01 = acos((IJ^2+L1^2-Ls0^2)/(2*IJ*L1)); % angle between L1 and IJ
    beta3 = acos((IJ^2+IC0^2-JC0^2)/(2*IJ*IC0)); % angle between IJ and IC0
    beta2 = ang-(beta3-beta01); % angle between L2 and IC0
    
   % pause
    
    %d1 = sqrt(L1^2+IC0^2-2*cos(ang+beta2)*L1*IC0); % distance between damper side hole to LBJ
    d2 = sqrt(L2^2+IJ^2-2*cos(ang+beta01)*L2*IJ); % distance between pullrod side hole to damper mount
    pullrod = sqrt(L2^2 + IC0^2 - 2*cos(beta2)*L2*IC0); % pullrod length
    RH02 = tetra2(J,C0,I,d2,pullrod,L2); % initial pullrod mount on rocker location
%     J
%     C0
%     I
%     d2
%     pullrod
%     L2
    %pause
    vec_rot = RH02 - I; % the vector from the mounting point to RH02
    orthogonality = dot(vec_rot,axis); % should be zero or near zero as the two are orthogonal

    
    %pause
    n = 21; % number of points
    travel = 64; % damper total travel in mm
    l_inc = travel/(n-1); % llength change of the damper
    
    if L1 + IJ < (Ls0 + travel/2)
        display('invalid geometry, L1 too short')
        bump = 1:n;
        IR = 1:n;
    else
        if L1 + (Ls0-travel/2) < IJ
            display('invalid geometry, L1 too short')
            bump = 1:n;
            IR = 1:n;
            
        else
            if  (Ls0-travel/2) + IJ < L1
                display('invalid geometry, L1 too long')
                bump = 1:n;
                IR = 1:n;
            else
                
                
                % RH2 = tetra2(J,I,F0,d2,L2,pullrod); % location of the pullrod hole
                
                
                l = zeros(n,1);
                T = zeros(3,n);
                for i = 1:n;
                    l(i) = Ls0 + (i-(n+1)/2)*l_inc; % current damper length
                    
                    
                    beta1 = acos((IJ^2+L1^2-l(i)^2)/(2*IJ*L1)); % angle between L1 and IJ
                    d2 = sqrt(L2^2+IJ^2-2*cos(ang+beta1)*L2*IJ); % distance between pullrod side hole to damper mount
                    
                    % use the EULER parameter vector rotation matrix,
                    % Dynamics of multibody system P31
                    ang_turned = beta1-beta01; % angle turned of the vector vec_rot, watch the sign!
                    c0 = cos(ang_turned/2);
                    c1 = axis(1)*sin(ang_turned/2);
                    c2 = axis(2)*sin(ang_turned/2);
                    c3 = axis(3)*sin(ang_turned/2);
                    
                    % transformation matrix
                    matrix = [1-2*c2^2-2*c3^2, 2*(c1*c2-c0*c3), 2*(c1*c3+c0*c2);
                              2*(c1*c2+c0*c3), 1-2*c1^2-2*c3^2, 2*(c2*c3-c0*c1);
                              2*(c1*c3-c0*c2), 2*(c2*c3+c0*c1), 1-2*c1^2-2*c2^2];
                    
                    vec_rot_new = matrix*vec_rot;
                    orthogonality1 = dot(vec_rot_new,axis);
                    orthogonality2 = dot(vec_rot,axis);
                    RH2 = I + vec_rot_new;
                    
                    % now find the distance between the origiinal OBJ location F0 and the
                    % new RH2 location, use this to determine RH2
                    d3 = sqrt(L2^2+IC0^2-2*cos(ang-beta3+beta1)*L2*IC0);
                    RH2_test = tetra2(J,I,C0,d2,L2,d3);
                    
                    
                    
                    orthogonality_test = dot(RH2-I,axis);
                    
                    
                    
                    C = tetra2(RH2,B,A,pullrod,BC,AC);
                    
                    
                    % find the location of the upper spherical joint C using the self
                    % defined function tetra2.m
                    F = tetra2(D,E,C, DF,EF,CF);
                    
                    % find the location of new trackrod pickup point on the upright
                    H = tetra2(C,G,F,CH,GH,FH);
                    
                    % find the new location of the tyre contact patch
                    
                    T(:,i) = tetra2(C,H,F, CT,HT,FT);
                    
                    % for the front pull rod system, it is difficult to
                    % determine the upright rotation about the kingpin. So
                    % It's a good aproximation using the UBJ vertical
                    % movement instead of calculating the vertical movement
                    % of the contact patch (they've got the almost the same horizontal swing arm)
                    %T(:,i) = C;
                    
                    %pause
%                     CT;
%                     FT;
%                     HT;
%                     C;
%                     F;
%                     H;
                    %pause
                    
                end
                
                bump = T(2,:); % in solidworks coordinate, y is the bump
                
                %bump = T(2,:)-362;
                
                IR = zeros(n,1);
                
                for k = 1:n-1; % so IR will have one point less than other graphs
                    IR(k) = (l(k)-l(k+1))/(bump(k+1)-bump(k));
                    
                end
                
                IR(n) = IR(n-1)+ (IR(n-1)-IR(n-2)); % simply approximate the last point by intepretation to equalise the size of the matrix
                
                plot(bump,IR)
            end
        end
       
    end
    grid on
    xlabel('bump/mm')
    ylabel('installation ratio')
    %% testing and debug plots
    %     figure
    %     scatter3(testF(1,:),testF(2,:),testF(3,:))
    %     xlabel('longitudinal')
    %     ylabel('lateral')
    %     zlabel('vertical')
    %
    %
    %     figure
    %     scatter3(T(1,:),T(2,:),T(3,:))
    %     xlabel('longitudinal')
    %     ylabel('lateral')
    %     zlabel('vertical')
    
    %display('all successful')