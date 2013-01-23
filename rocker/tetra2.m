%%
% 
%
% This function finds the coordinate of the point C on a tetrahedron ABFC
% with given coordinates of A, B and F and the distance from these three
% points to C. (ie, like finding the intersection of three spheres) 
%
% This function can be called with 
%
%  c = tetra2(A,F,B,AC,CF,BC)
%
% The required inputs are the coordinates of A, F, B in vector form and the
% distance from A, B, F to C. 
%
% There are two potential solutions of the position of C on either side of 
% the plane defined by ABF.
% Using the rotation in the order of ABF and right hand grip rule gives the
% direction to which side of the surface C locates
%
% Author: Yunlong Xu, Full Blue Racing

% Last updated: 16/08/2011
%
% this function works even if the point needs to be located is in the same
% plane with the other three points
%
% 14/04/2010 updaded, to solve some problems in the older version. more
% vector identities are used.
%
% 16/08/2011 update. Added documentation


function C = tetra2(A,F,B,AC,CF,BC)
    
    
    % find the length of AB, AF % BF
    AB = norm(A-B);
    AF = norm(A-F);
    BF = norm(B-F);
    
    % construct a line Cp that is perpendicular to the plane ABF and
    % intersects the plane at the point p.
    
    % construct a line through p that is perpendicular to AF and intersects
    % AF at t. Do similar trick to construct ps perpendicular to BF with s on BF
    
%     if AC>CF+AF||CF>AC+AF||AF>AC+CF||CF>BF+BC||BF>CF+BC||BC>CF+BF||BF>AF+AB||AF>BF+AB||AB>AF+BF
%         display('invalid tetrahedron!')
        %C = 10000*[1,1,1]';
    %else
        
        CFA = acos((CF^2+AF^2-AC^2)/(2*AF*CF)); % find angle CFA
        tF = CF*cos(CFA); % find tF
        t = (A-F)*tF/AF + F; % find the position vector of t
        FB_hat = (B-F)/norm(B-F); % unit vector in the FB direction
        
        % find s
        CFB = acos((CF^2+BF^2-BC^2)/(2*CF*BF)); % find angle CFB
        sF = CF*cos(CFB); % find SF
        s = (B-F)*sF/BF + F; % find the position vector of s
        FA_hat = (A-F)/norm(A-F); % unit vector in the FA direction
        
        
        % assume the position of p is at F + x*FA_hat+ y*FB_hat
        
        % use pt perpendicular to AF and ps perpendicular to BF
        
        transform_matrix = [1,dot(FA_hat,FB_hat);dot(FA_hat,FB_hat),1];
 
        
        xy = (transform_matrix)\[tF;sF]; % use right division instead of inverse to increase speed
 
        
        p = F + xy(1)*FA_hat+ xy(2)*FB_hat;
        
        
        
        
%         % now extend tp to intersect BF at q and extend sp to intersect AF at g
%         
%         % find q
%         BFA = acos((BF^2 +AF^2 - AB^2)/(2*BF*AF)); % find the angle BFA
%         qF = tF/cos(BFA);
%         %q = (B-F)*qF/BF + F; % find position vector of q
%         
%         % find ps
%         qs = qF-sF;
%         ps = qs*tan(pi/2 - BFA); % length of ps
%         
%         % find g
%         gF = sF/cos(BFA);
%         g = (A-F)*gF/AF + F;  % position vector of g
        
%         % now we can find p
%         sp_unit = (g-s)/norm(g-s); % find the unit vector of sp
%         p = s + sp_unit*ps;
%         
        % find Cp
        pF = norm(p-F); % find the length of pF
        Cp = sqrt(CF^2 - pF^2); % find length of Cp using pythagoras, CpF = 90 deg
        
        % find position of c, the upper spherical joint
        pC_unit = cross((F-B),(F-A))/norm(cross((F-B),(F-A))); % find the unit vector normal to the plane ABF
        % NOTE! for cross product, the ORDER matters!
        
        C = p + pC_unit*Cp;
        
        
%     end
    
    
