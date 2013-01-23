clear all
close all
% File for calculating steering angles and percentage Ackermanns of a
% steering system based on the geometry

%Pete Hall FBR 2013

%Define Geometry and features (mm)
A = [-300,0];          % Left Hand Ball Joint Position
B = [-600,80];         % Left Hand Upright Attachment
C = [-600,100];        % Left Hand Upright Centre of Rotation
D = [300,0];           % Right Hand Ball Joint Position
E = [600,80];          % Right Hand Upright Attachment
F = [600,100];         % Right Hand Upright Centre of Rotation
Travel = 35;           % Steering Rack Travel value (lock to lock) (mm)
LHVector = C-B;        % Calculate Vector of LH wheel direction in middle
RHVector = F-E;        % Calculate Vector of RH wheel direction in middle
Speed = 50;            % Rack Speed (mm/rev)
Speed = Speed/360;

%Define Ackermanns measurement type
CornerRadius = 8000;    %Define corner radius that Ackermanns percentage will be calculated at
Track = F(1) - C(1);    %Track
Wheelbase = 2000;       %Input Car wheelbase


AB = norm(A-B);        % Track Rod Length
BC = norm(B-C);        % Steering Arm Length

n=1000;                % Number of iterations



Tetra1 = C; Tetra1(3)=50;
Tetra2 = C; Tetra2(3)=-50;
Tetra3 = F; Tetra3(3)=50;
Tetra4 = F; Tetra4(3)=-50;
Tetraradius = (BC^2 + 50^2)^0.5;

%Test to ensure all travel can actually be used
Maxlength = AB+BC;
Balljointdistance = (Maxlength^2 - (C(2)-A(2))^2)^0.5;
Maxtravel = 2*(Balljointdistance-(A(1)-C(1)));
if (Travel/2)>(Balljointdistance-(A(1)-C(1)))
    display('Travel is too large, will contain complex solutions')
    display(Maxtravel)
else
end
Ackermanninner = 90 - atand((CornerRadius-(Track/2))/Wheelbase);
Ackermannouter = 90 - atand((CornerRadius+(Track/2))/Wheelbase);
Ackermanndifference = Ackermanninner - Ackermannouter;

%General Sweep for All Rack Positions
for i=1:n
    LHBall = A - [Travel/2,0] + [Travel*(i-1)/(n-1),0]; %Sweeping through to find ball joint positions
    RHBall = D - [Travel/2,0] + [Travel*(i-1)/(n-1),0];
    
    Wheelangle(i) = (LHBall(1) - A(1))/Speed;          %Steering wheel angle in degrees (Left defined as -ve)
    Steeringpercentage(i) = -100+ 200*(i-1)/(n-1);    %Define a % of steering lock used (Left Lock -100%, Right Lock 100%)
    
    LHBall(3)=0;    RHBall(3)=0;
    B3D = tetra2(LHBall,Tetra1,Tetra2,AB,Tetraradius,Tetraradius);      %Finding the new upright attachment points by utilising function tetra2
    E3D = tetra2(RHBall,Tetra4,Tetra3,AB,Tetraradius,Tetraradius);
    B(1) = B3D(1); B(2) = B3D(2); E(1) = E3D(1); E(2) = E3D(2);
    
    LHwheelvector = C-B;            %Create new wheel vectors
    RHwheelvector = F-E;
    
    LHwheelangle(i) = acosd(dot(LHwheelvector,LHVector)/(norm(LHwheelvector)*norm(LHVector)));
    RHwheelangle(i) = acosd(dot(RHwheelvector,RHVector)/(norm(RHwheelvector)*norm(RHVector)));   %Find angle from original position
    
end
%Check for Ackermanns Position
B = [C(1)+BC*sind(Ackermanninner),C(2) - BC*cosd(Ackermanninner)];  %Use Ackermann angle to find point of upright attachment
Distancex = (AB^2-(B(2)-A(2))^2)^0.5;                               %Calculate the new position of Ball joint
Movement = (B(1)+ Distancex) - A(1);                                %Calculate movement of rack
D(1) = D(1) + Movement;                                             %Find effect on outer wheel
D(3) = 0;
E3D = tetra2(D,Tetra4,Tetra3,AB,Tetraradius,Tetraradius);
E(1) = E3D(1); E(2) = E3D(2);
RHwheelvector = F-E;
OuterAngle = acosd(dot(RHwheelvector,[0,1])/(norm(RHwheelvector)));
ActualDifference = -OuterAngle+Ackermanninner;
PercentageAckermanns = ActualDifference*100/Ackermanndifference;

display('Percentage Ackermann is')
display(PercentageAckermanns)


Wheeldifference=LHwheelangle-RHwheelangle;

figure
plot(Wheelangle,LHwheelangle)

figure
plot(Wheelangle,RHwheelangle)

figure
plot(Wheelangle, Wheeldifference)


    