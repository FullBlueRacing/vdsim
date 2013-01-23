function [ x y gradAB gradCD] = lineintersect(A,B,C,D)
%UNTITLED Summary of this function goes here
% calculates the coordinate of the intersection of AB and CD
%   Detailed explanation goes here


% find equation for line AB
gradAB = (B(2)-A(2))/(B(1)-A(1));
interceptAB = A(2)-gradAB*A(1);

% find equation for line CD
gradCD = (D(2)-C(2))/(D(1)-C(1));
interceptCD = C(2) - gradCD*C(1);

% test if the two lines are parallel
if gradAB == gradCD
    % lines are parallel
    disp('parallel lines! no intersection!')
    x = NaN;
    y = NaN;
   % ycheck = NaN;
else
    
    % now find the point of intersection
    
    x = -(interceptCD-interceptAB)/(gradCD-gradAB);
    y = x*gradAB +interceptAB;
    %ycheck = x*gradCD+interceptCD;
    
end
end

