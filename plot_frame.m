function [outputArg1,outputArg2, outputArg3] = plot_frame(position,rotation,scale)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    position
    rotation
    scale
end

arguments (Output)
    outputArg1
    outputArg2
    outputArg3
end

plot3(position(1),position(2),position(3))
hold on;
quiver3(position(1),position(2),position(3),rotation(1,1),rotation(2,1),rotation(3,1),0.1*scale,"r");
quiver3(position(1),position(2),position(3),rotation(1,2),rotation(2,2),rotation(3,2),0.1*scale,"g");
quiver3(position(1),position(2),position(3),rotation(1,3),rotation(2,3),rotation(3,3),0.1*scale,"b");

outputArg1 = position;
outputArg2 = rotation;
outputArg3 = scale;
end