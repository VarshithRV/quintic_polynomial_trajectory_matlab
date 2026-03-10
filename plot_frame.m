function [outputArg1,outputArg2, outputArg3] = plot_frame(position,rotationq,scale)
% plots a small frame given position and rotation in quaternion
arguments (Input)
    position
    rotationq quaternion
    scale
end

arguments (Output)
    outputArg1
    outputArg2
    outputArg3
end

rotation = rotationq.rotmat('frame');
grid on;
axis equal;
plot3(position(1),position(2),position(3))
hold on;
quiver3(position(1),position(2),position(3),rotation(1,1),rotation(2,1),rotation(3,1),0.1*scale,"r");
quiver3(position(1),position(2),position(3),rotation(1,2),rotation(2,2),rotation(3,2),0.1*scale,"g");
quiver3(position(1),position(2),position(3),rotation(1,3),rotation(2,3),rotation(3,3),0.1*scale,"b");

outputArg1 = position;
outputArg2 = rotation;
outputArg3 = scale;
end