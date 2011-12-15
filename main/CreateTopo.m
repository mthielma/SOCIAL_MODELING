% function [XTopo,YTopo,ZTopo] = CreateTopo(Parameter,BeachWidth,BeachHeight,MaxElevation)

% Create topography

[XTopo,YTopo] = meshgrid(Parameter.xmin:Parameter.xmax,Parameter.ymin:Parameter.ymax);

% create building topo
DomainWidth = (Parameter.xmax-Parameter.xmin)-BeachWidth;
DomainHeight = (Parameter.ymax-Parameter.ymin)-BeachWidth;

mx = (MaxElevation-BeachHeight)/DomainWidth;
my = (MaxElevation-BeachHeight)/DomainHeight;

[x_bt,y_bt] = meshgrid(0:DomainWidth,0:DomainHeight);

z_bt = mx*x_bt + my*y_bt + BeachHeight;

x_bt = x_bt + BeachWidth;
y_bt = y_bt + BeachWidth;


x1 = Parameter.xmin:Parameter.xmax;
y1 = 0*x1;
z1 = 0*x1;

y2 = Parameter.ymin:Parameter.ymax;
x2 = 0*y2;
z2 = 0*y2;

x = [x_bt(:);x1';x2'];
y = [y_bt(:);y1';y2'];
z = [z_bt(:);z1';z2'];

z = max(-0.4,z);

ZTopo = 0*XTopo;
ZTopo(:) = griddata(x,y,z,XTopo,YTopo);

