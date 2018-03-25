
function select_point
    close all
    clearvars
    clc
    fig = figure;
    ax = scatter3([1 2 3],[2 5 4],[4 5 8]);
    set(ax,'Tag','ax_tag');
    datacursormode(fig)
    dcm_obj = datacursormode(fig);
    set(dcm_obj,'UpdateFcn',@myfunction)
    java_slider
end

function output_txt = myfunction(~,event_obj)
    event_obj.Position    %Object containing event data structure
    output_txt = 'Hola';   %Data cursor text
end

function java_slider

    % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
    jSlider1 = javax.swing.JSlider;
    javacomponent(jSlider1,[10,70,200,50]);
    set(jSlider1, 'Value',84, 'MajorTickSpacing',20, 'MinorTickSpacing',5,'PaintLabels',true, 'PaintTicks',true);  % with labels, no ticks
    hjSlider1 = handle(jSlider1, 'CallbackProperties');
    set(hjSlider1, 'StateChangedCallback', @myCallback1);  %alternative

    function myCallback1(hjSlider1,~)
        disp(get(hjSlider1,'Value'));
    end

    % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
    jSlider2 = javax.swing.JSlider;
    javacomponent(jSlider2,[250,200,50,200]);
    set(jSlider2, 'Value',84, 'MajorTickSpacing',20,'Orientation',jSlider2.VERTICAL, 'MinorTickSpacing',5, 'PaintLabels',true, 'PaintTicks',true);  % with labels, no ticks
    hjSlider2 = handle(jSlider2, 'CallbackProperties');
    set(hjSlider2, 'StateChangedCallback', @myCallback2);  %alternative

    function myCallback2(hjSlider2,~)
        ax = findobj(get(gca,'Children'),'Tag','ax_tag');
        set(ax,'XData',[1 get(hjSlider2,'Value') 3]);
    end

end