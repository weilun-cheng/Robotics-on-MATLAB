function tmouse(action)
 
global h
if nargin == 0, action = 'start'; end
switch(action)
    case 'start',
        x=5:30;
        y=x.^2-40.*x+400;
        z=x.^2-40.*x+400;
        plot3(x,y,z);
        axis([5,30,-50,250]);
        title('Move your mouse !'); 
        set(gcf,'WindowButtonMotionFcn','tmouse move');
        h = text(2,-80,' ');
  case 'move',
        currPt = get(gca, 'CurrentPoint');
        x = currPt(1,1);
        y = currPt(1,2);
        set(h,'String',[num2str(x),',',num2str(y)]);
 end