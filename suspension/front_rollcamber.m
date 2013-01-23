clear all
close all

% this script simulates the camber change in roll for pick-up point design
% Edited December 2012 for ARB by Peter Hall
% define initial geometry
% NB - This program neglects track change in pure roll?


% This program is written for 2D roll geometry yet uses the function tetra2
% (a 3D function) to calculate 2D geometry movements. This works by using 2
% 3D points specified as out of the plane in equal distance hence creating
% the intersection of a sphere with a circle that exists in the 2D plane we
% are interested in.


% left hand side
B = [-590,136]; % LBJ
C = [-573,370]; % UBJ
A = [-612,0]; % contact patch
D = [-260,280]; % upper pick up
E = [-210,122]; % lower pick up
K = [-551,170]; % Push Rod - Upright Joint
L = [-216.5,532];   % Push Rod - Rocker Joint
M = [-187,505];   % Rocker Mounting Point
N = [-227,505];   % ARB Attachment Point
O = [-227,100];   % ARB End Centre Line Point

% right hand side
F = [210,122]; % lower pick up
G = [260,280]; % upper pick up
H = [573,370]; % UBJ
I = [590,136]; % LBJ
J = [612,0]; % contact patch
P = [551,170];  % Push Rod - Upright Joint
Q = [216.5,532];      % Push Rod - Rocker Joint
R = [187,505];      % Rocker Mounting Point
S = [227,505];      % ARB Attachment Point
T = [227,100];      % ARB End Centre Line Point

figure
frame = [A;B;C;D;E;F;G;H;I;J;K;L;M;N;O;P;Q;R;S;T];
scatter(frame(:,1),frame(:,2));
axis equal
grid on  

%Specify 3D geometry for out of plane lever arm  movement
%Left Hand Side
U = N; U(3)=-40; %Specify 3D offset of ARB Attachment Point
V = O; V(3)=0; %Specify 3D offset of ARB Center Line End Point
W = [-227,100,-80];   %Specify 3D position of Lever arm End

%Right Hand Side
X = S; X(3)=-40; %Specify 3D offset of ARB Attachment Point
Y = T; Y(3)=0; %Specify 3D offset of ARB Center Line End Point
Z = [227,100,-80];   %Specify 3D position of Lever arm End

% define linkage length
BE = norm(B-E); % lower wishbone
CD = norm(C-D); % upper wishbone
AB = norm(A-B);
IJ = norm(I-J);
KL = norm(K-L); % Push Rod Length
LM = norm(L-M); % Rocker Mounting to push rod joing distance
MN = norm(M-N); % Rocker Mounting to ARB attachment distance
LN = norm(L-N); % Push Rod to ARB Attachment distance
UW = norm(U-W); % ARB Suspension Member Length
VW = norm(V-W); % ARB Lever Arm Length

%Polar Co-ordinates of Push Rod Attachment Relative to Contact Patch
[LHSAngle,LHSDistance] = cart2pol(K(1)-A(1),K(2)-A(2));
[RHSAngle,RHSDistance] = cart2pol(P(1)-J(1),P(2)-J(2));

% define camber baseline
aCamberBasedeg = atan((I(2)-J(2))/(J(1)-I(1)))/pi*180;

% now do the calculations
n = 1000;

%Initial Angle of Rockers
LHRockerInitial = atan((L(2)-M(2))/(L(1)-M(1)));
RHRockerInitial = atan((Q(2)-R(2))/(Q(1)-R(1)));
[MtoNangle,MtoN] = cart2pol(N(1)-M(1),N(2)-M(2));
[RtoSangle,RtoS] = cart2pol(S(1)-R(1),S(2)-R(2));

%Specify Special Points 50mm either side of Center Line End Point so can
%use tetra2 to find moved lever points
ARB1 = O + [50,0];
ARB2 = O + [-50,0];
ARB3 = T + [50,0];
ARB4 = T + [-50,0];
Tetraradius = sqrt(50^2+VW^2);

for i = 1:n;
    angdeg = (i-1)*0.004/n;
    angrad = angdeg/180*pi;
    
    % find the instaneous centres for the two sides
    
    [ICLx,ICLy] = lineintersect(D,C,B,E);
    [ICRx,ICRy] = lineintersect(G,H,F,I);
   
    ICL = [ICLx,ICLy];
    ICR = [ICRx,ICRy];
    
    % now find the instantaneous centre of rotation
    
    [RCx,RCy] = lineintersect(ICL,A,ICR,J);
    
    RC = [RCx,RCy];
    x(i) = RCx;
    y(i) = RCy;
    % now calculate the rotation of the sprung mass
    E_RC = E-RC;
    [angERC,ERC] = cart2pol(E_RC(1),E_RC(2));
    angERC = angERC+angrad;
    [E_RC(1),E_RC(2)] = pol2cart(angERC,ERC);
    E = RC+E_RC;
    
    D_RC = D-RC;
    [angDRC,DRC] = cart2pol(D_RC(1),D_RC(2));
    angDRC = angDRC+angrad;
    [D_RC(1),D_RC(2)] = pol2cart(angDRC,DRC);
    D = RC+D_RC;
    
    G_RC = G-RC;
    [angGRC,GRC] = cart2pol(G_RC(1),G_RC(2));
    angGRC = angGRC+angrad;
    [G_RC(1),G_RC(2)] = pol2cart(angGRC,GRC);
    G = RC+G_RC;
    
    F_RC = F-RC;
    [angFRC,FRC] = cart2pol(F_RC(1),F_RC(2));
    angFRC = angFRC+angrad;
    [F_RC(1),F_RC(2)] = pol2cart(angFRC,FRC);
    F = RC+F_RC;
    
    M_RC = M-RC;
    [angMRC,MRC] = cart2pol(M_RC(1),M_RC(2));
    angMRC = angMRC+angrad;
    [M_RC(1),M_RC(2)] = pol2cart(angMRC,MRC);
    M = RC+M_RC;
    
    O_RC = O-RC;
    [angORC,ORC] = cart2pol(O_RC(1),O_RC(2));
    angORC = angORC+angrad;
    [O_RC(1),O_RC(2)] = pol2cart(angORC,ORC);
    O = RC+O_RC;
    
    R_RC = R-RC;
    [angRRC,RRC] = cart2pol(R_RC(1),R_RC(2));
    angRRC = angRRC+angrad;
    [R_RC(1),R_RC(2)] = pol2cart(angRRC,RRC);
    R = RC+R_RC;
    
    T_RC = T-RC;
    [angTRC,TRC] = cart2pol(T_RC(1),T_RC(2));
    angTRC = angTRC+angrad;
    [T_RC(1),T_RC(2)] = pol2cart(angTRC,TRC);
    T = RC+T_RC;
    
    %Rotation of positional points on ARB used in the tetra2 function to
    %find ARB twist later on in the script.
    ARB1_RC = ARB1-RC;
    [angARB1RC,ARB1RC] = cart2pol(ARB1_RC(1),ARB1_RC(2));
    angARB1RC = angARB1RC+angrad;
    [ARB1_RC(1),ARB1_RC(2)] = pol2cart(angARB1RC,ARB1RC);
    ARB1 = RC+ARB1_RC;
    
    ARB2_RC = ARB2-RC;
    [angARB2RC,ARB2RC] = cart2pol(ARB2_RC(1),ARB2_RC(2));
    angARB2RC = angARB2RC+angrad;
    [ARB2_RC(1),ARB2_RC(2)] = pol2cart(angARB2RC,ARB2RC);
    ARB2 = RC+ARB2_RC;
    
    ARB3_RC = ARB3-RC;
    [angARB3RC,ARB3RC] = cart2pol(ARB3_RC(1),ARB3_RC(2));
    angARB3RC = angARB3RC+angrad;
    [ARB3_RC(1),ARB3_RC(2)] = pol2cart(angARB3RC,ARB3RC);
    ARB3 = RC+ARB3_RC;
    
    ARB4_RC = ARB4-RC;
    [angARB4RC,ARB4RC] = cart2pol(ARB4_RC(1),ARB4_RC(2));
    angARB4RC = angARB4RC+angrad;
    [ARB4_RC(1),ARB4_RC(2)] = pol2cart(angARB4RC,ARB4RC);
    ARB4 = RC+ARB4_RC;
    
    bottom = F-E;
    aRollrad = cart2pol(bottom(1),bottom(2));
    aRolldeg(i) = aRollrad/pi*180;
    
    % calculate LHS camber
    AE = norm(E-A);
    FJ = norm(F-J);
    
    angEAB = acos((AE^2+AB^2-BE^2)/(2*AE*AB));
    angEAJ = atan((E(2)-A(2))/(E(1)-A(1)));
    
    aCamberLrad = angEAB +angEAJ;
    aCamberLdeg(i) = aCamberLrad/pi*180-aCamberBasedeg;
    
    
    angIJF = acos((IJ^2+FJ^2-BE^2)/(2*IJ*FJ));
    angIJA = atan((F(2)-J(2))/(J(1)-F(1)));
    
    aCamberRrad = angIJF+angIJA;
    aCamberRdeg(i) = aCamberRrad/pi*180-aCamberBasedeg;
    
    %Find new upright push rod positions based on the camber change of the
    %wheel and assumption of 0 track change
    Leftangle=LHSAngle+(aCamberLdeg(i)*2*pi/360);
    [Leftmovex,Leftmovey]=pol2cart(Leftangle,LHSDistance);
    K=A+[Leftmovex,Leftmovey];
   
    %Check +ve and -ve signs for left angle and right angle!!!!!
    Rightangle=RHSAngle-(aCamberRdeg(i)*2*pi/360);
    [Rightmovex,Rightmovey]=pol2cart(Rightangle,RHSDistance);
    P=J+[Rightmovex,Rightmovey];
    
    %We now have the locations of all the points fixed to the chassis and
    %the wheel, we need to find the push rod attachment point on the rocker
    %and then the ARB attachment point on the rocker. To do this we use
    %tetra2 function with 2 points the same, specified slightly into and
    %out of the plane to give the circle we require. 
    
    %LHS Push Rod Rocker Attachment Position
    Variable1 = M; Variable2 = M; Variable1(3) = -50; Variable2(3) = 50;
    Length = sqrt(50^2+LM^2);
    Variable3 = K; Variable3(3)=0;
    L3D = tetra2(Variable2,Variable1,Variable3,Length,Length,KL);
    L(1)=L3D(1); L(2) = L3D(2);
    
    %RHS Push Rod Rocker Attachment Position
    Variable1 = R; Variable2 = R; Variable1(3) = -50; Variable2(3) = 50;
    Variable3 = P; Variable3(3)=0;
    Q3D = tetra2(Variable1,Variable2,Variable3,Length,Length,KL);
    Q(1)=Q3D(1); Q(2)=Q3D(2);
    %Push-rod attachment point found
    
    %Now we have the 2 rocker points, we can find the third from a similar
    %method
    %LHS ARB Rocker Attachment Position
    Variable1 = M; Variable2 = M; Variable1(3) = -50; Variable2(3) = 50; Variable3 = L; Variable3(3)=0;
    Length = sqrt(50^2+MN^2);
    N3D = tetra2(Variable1,Variable2,Variable3,Length,Length,LN);
    N(1)=N3D(1); N(2)=N3D(2);
    
    %RHS ARB Rocker Attachment Position
    Variable1 = R; Variable2 = R; Variable1(3) = -50; Variable2(3) = 50; Variable3 = Q; Variable3(3)=0;
    S3D = tetra2(Variable2,Variable1,Variable3,Length,Length,LN);
    S(1)=S3D(1); S(2)=S3D(2);
   
    %Now we do a 3D use of tetra2 function. We find the new position of the
    %lever arm by considering the intersection of a sphere with a circle.
    %We have to define the circle to lie in the rotated plane of the lever,
    %this is done by moving ARB points defined earlier with the sprung
    %mass.
    ARB13D = ARB1; ARB13D(3)=0;
    ARB23D = ARB2; ARB23D(3)=0;
    ARB33D = ARB3; ARB33D(3)=0;
    ARB43D = ARB4; ARB43D(3)=0;
    U=N; U(3)=40;
    X=S; X(3)=40;
    
    W = tetra2(U,ARB23D,ARB13D,UW,Tetraradius,Tetraradius);
    Z = tetra2(X,ARB43D,ARB33D,UW,Tetraradius,Tetraradius);
    %Lever Points found by tetra2 function
    
    % Set 3D Points for centre position
    V(1) = O(1); V(2) = O(2);
    Y(1) = T(1); Y(2) = T(2);
    
    %Find Vectors of Lever Arms
    ARBVector1 = W-V;
    ARBVector2 = Z-Y;
    
    %Find angle between vectors using dot product relationship
    Twistangle = acosd(dot(ARBVector1,ARBVector2)/(norm(ARBVector1)*norm(ARBVector2)));
    ARBTwist(i)= Twistangle;
    
    %Fill in vectors to plot graphs of position changes. Just to check that
    %we have tetra2 function selecting answer that lies in the correct
    %position
    Lx(i)=L(1);Ly(i)=L(2);
    Mx(i)=M(1);My(i)=M(2);
    Nx(i)=N(1);Ny(i)=N(2);
    Ox(i)=O(1);Oy(i)=O(2);
    Qx(i)=Q(1);Qy(i)=Q(2);
    Rx(i)=R(1);Ry(i)=R(2);
    Sx(i)=S(1);Sy(i)=S(2);
    Tx(i)=T(1);Ty(i)=T(2);
 
end
 
%Find Linear Line of Best fit for Motion Ratio
MR = polyfit(aRolldeg,ARBTwist,1);
display('ARB Motion Ratio for given set up is ')
display(MR(1))

figure
frame = [A;B;C;D;E;F;G;H;I;J;K;L;M;N;O;P;Q;R;S;T];
scatter(frame(:,1),frame(:,2));
axis equal
grid on   

figure
plot(x,y,'rx')
grid on
xlabel('x-axis (mm)')
ylabel('y-axis (mm)')
title('front roll centre migration')

figure
plot(aRolldeg,aCamberLdeg,'r',aRolldeg,aCamberRdeg,'b');
legend('front left','front right','location','best')
grid on
xlabel('body roll angle (deg)')
ylabel('camber change (deg)')
title('front camber change in roll')



figure
plot(aRolldeg,ARBTwist)

figure
plot(Lx,Ly,'b',Mx,My,'r',Nx,Ny,'y',Ox,Oy,'g',Qx,Qy,'b',Rx,Ry,'r',Sx,Sy,'y',Tx,Ty,'g')

