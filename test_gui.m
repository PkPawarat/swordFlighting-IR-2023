

simple_gui();

function simple_gui()
    % Create a figure
    screenSize = get(0, 'ScreenSize');
    fig = figure('Position', [0,0,screenSize(1,3),screenSize(1,4)], 'Name', 'Simple GUI');
    
    % Create a button
    btn = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Click Me', ...
        'Position', [100, 100, 100, 40], 'Callback', @buttonCallback);
end

function buttonCallback(~, ~)
    msgbox('Button Clicked!', 'Message', 'modal');
end
