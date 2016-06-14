function mainFunc
hFigure= figure;
[x,y]=meshgrid(linspace(-15,15));%设定xy范围
 z=sin((x.^2+y.^2).^0.5)./((x.^2+y.^2).^0.5);
surf(x,y,z);%此处画图
hCursor= datacursormode(hFigure)
set(hCursor,'UpdateFcn',@myFunc)


