function[x,y]=func(a,b,c,d);
slope1=(a(2)-c(2))/(a(1)-c(1));
slope2=(b(2)-d(2))/(b(1)-d(1));
inter1= a(2)-slope1*a(1);
inter2= b(2)-slope2*b(1);

x=(inter1-inter2)/(slope2-slope1);
y=inter1+slope1*x;

end