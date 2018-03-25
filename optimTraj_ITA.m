function optimTraj_ITA(WP)

    ita_fig = figure('Visible','off','Resize','off','Position',[760,88,700,500]);  %  Create and then hide the UI as it is being constructed.
    ita_fig.MenuBar = 'none'; % Deactivate Menu Bar of the figure
    ita_fig.NumberTitle = 'off'; % Deactivate the label "Figure n" in the title bar
    ita_fig.Name = 'Initial Trajectory Assistant'; % Assign the name to appear in the GUI title.
    movegui(ita_fig,'northeast') % Move the GUI.
    
    plot3D_ax = axes('Position',[0.1 0.25 0.7 0.7]);
    hold on
    plot3D = scatter3(plot3D_ax,WP.north,WP.east,WP.up,9,'b','filled')
    hold off
    grid
%     title(['Total time ' num2str(propagatedState.totalTime) 's'])
    axis equal
    axis vis3d % Lock aspect ratio of axes
    view(-45,30); % Azimuth and elevation of initial view (degrees)
    xlabel('North')
    ylabel('East')
    zlabel('Up')
    set(plot3D,'Tag','plot3D_tag');
    datacursormode(ita_fig)
    dcm_obj = datacursormode(ita_fig);
    set(dcm_obj,'UpdateFcn',@myfunction)
    java_slider
    
    
    ita_fig.Visible = 'on'; % Make GUI visible.
    
end

function output_txt = myfunction(~,event_obj)
    event_obj.Position    %Object containing event data structure
    output_txt = 'Hola';   %Data cursor text
end

function java_slider

    % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
    jSlider1 = javax.swing.JSlider;
    javacomponent(jSlider1,[400,20,200,50]);
    set(jSlider1, 'Value',84, 'MajorTickSpacing',20, 'MinorTickSpacing',5,'PaintLabels',true, 'PaintTicks',true);  % with labels, no ticks
    hjSlider1 = handle(jSlider1, 'CallbackProperties');
    set(hjSlider1, 'StateChangedCallback', @myCallback1);  %alternative

    function myCallback1(hjSlider1,~)
        disp(get(hjSlider1,'Value'));
    end

    % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
    jSlider2 = javax.swing.JSlider;
    javacomponent(jSlider2,[640,20,50,200]);
    set(jSlider2, 'Value',84, 'MajorTickSpacing',20,'Orientation',jSlider2.VERTICAL, 'MinorTickSpacing',5, 'PaintLabels',true, 'PaintTicks',true);  % with labels, no ticks
    hjSlider2 = handle(jSlider2, 'CallbackProperties');
    set(hjSlider2, 'StateChangedCallback', @myCallback2);  %alternative

    function myCallback2(hjSlider2,~)
        plot3D = findobj(get(gca,'Children'),'Tag','plot3D_tag');
        disp(get(hjSlider2,'Value'));
%         set(ax,'XData',[1 get(hjSlider2,'Value') 3]);
    end

end