function demoOnImageClick
    clf;
    clear;
    axis([-2,2,-2,2]);
    grid on;

    set(gca,'ButtonDownFcn',@PlotClickCallback)
    
end

function PlotClickCallback ( objectHandle , eventData )

    coordinates = get(objectHandle,'CurrentPoint'); 
    coordinates = coordinates(1,1:2);

    plotCoords(coordinates);
end

function plotCoords(coordinates)

    initial_ang_pos = atan2(coordinates(2),coordinates(1));
    final_ang_pos = pi/2;
    ang_pos = initial_ang_pos;

    for n=1:100
        ang_pos = ang_pos+(final_ang_pos-initial_ang_pos)/100;
        XY_end_pos = [cos(ang_pos),sin(ang_pos)];
        
        cla(gca);
        hold on;
        plot(XY_end_pos(1),XY_end_pos(2),'b.','MarkerSize',50);    
        plot([0,XY_end_pos(1)],[0,XY_end_pos(2)],'LineWidth',3);
        hold off;
        drawnow;
    end

end
