% Create a dynamic drawing of the small freight train
function Drawing(x, Train, flag)

% Let the values on the horizontal axis increase from right to left
set(gca, 'XDir', 'reverse')

% Model the locomotive and two wagons of the train as l x w rectangles
l = 1;
w = 1/3;

% Designate the vertices of the locomotive and two wagons
v = [x(1) + l / 2                                   0;
     x(1) + l / 2                                   w;
     x(1) - l / 2                                   w;
     x(1) - l / 2                                   0;
     x(1) - Train.L_0 - x(2) + l / 2                0;
     x(1) - Train.L_0 - x(2) + l / 2                w;
     x(1) - Train.L_0 - x(2) - l / 2                w;
     x(1) - Train.L_0 - x(2) - l / 2                0;
     x(1) - x(2) - 2 * Train.L_0 - x(3) + l / 2     0;
     x(1) - x(2) - 2 * Train.L_0 - x(3) + l / 2     w;
     x(1) - x(2) - 2 * Train.L_0 - x(3) - l / 2     w;
     x(1) - x(2) - 2 * Train.L_0 - x(3) - l / 2     0];  

% Determine the faces
f = [1  2  3  4;
     5  6  7  8;
     9  10 11 12];

% Plot the drawing
if flag == true
    c = [1 0 0;
         0 1 0;
         0 0 1];
    Train.Drawing = patch('Faces', f, 'Vertices', v, 'FaceVertexCData', c, 'FaceColor', 'flat');
    flag == false;
else
    set(Train.Drawing, 'Vertices', v);
end

% Define the current horizontal and vertical axis
axis([x(1) - 2 * Train.L_0 - l x(1) + l 0 5]);

% Draw everything
drawnow

end

