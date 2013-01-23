clear all
close all

% this script simulates the camber change in roll for pick-up point design

% define initial geometry

% left hand side
B = [-563.5,125]; % LBJ
C = [-563.5,375]; % UBJ
A = [-600,0]; % contact patch
D = [-240,290]; % upper pick up
E = [-230,125]; % lower pick up

% right hand side
F = [230,125]; % lower pick up
G = [240,290]; % upper pick up
H = [563.5,375]; % UBJ
I = [563.5,125]; % LBJ
J = [600,0]; % contact patch

% define linkage length
BE = norm(B-E); % lower wishbone
CD = norm(C-D); % upper wishbone
AB = norm(A-B);
IJ = norm(I-J);

% define camber baseline
aCamberBasedeg = atan((I(2)-J(2))/(J(1)-I(1)))/pi*180;

% now do the calculations
n = 1000;

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
    
end
    
 
figure
frame = [A;B;C;D;E;F;G;H;I;J];
scatter(frame(:,1),frame(:,2));
axis equal
grid on   

figure
plot(x,y,'rx')
grid on
xlabel('x-axis (mm)')
ylabel('y-axis (mm)')
title('rear roll centre migration')

figure
plot(aRolldeg,aCamberLdeg,'r',aRolldeg,aCamberRdeg,'b');
legend('rear left','rear right','location','best')
grid on
xlabel('body roll angle (deg)')
ylabel('camber change (deg)')
title('rear camber change in roll')
