function [x,y,z] = CreateSphere(r,XMid,YMid,ZMid)
 
phi=linspace(0,pi,30); 
theta=linspace(0,2*pi,40); 
[phi,theta]=meshgrid(phi,theta); 
x=r*sin(phi).*cos(theta)+XMid; 
y=r*sin(phi).*sin(theta)+YMid; 
z=r*cos(phi)+ZMid; 