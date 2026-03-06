function [X,Y,Z,delta_t] = quinticPolynomialTrajectory(initial_conditions,final_conditions,duration)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    
    arguments (Input)
        initial_conditions
        final_conditions
        duration
    end

    arguments (Output)
        X
        Y
        Z
        delta_t
    end
    
    t = duration;
    A = [
        1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0;
        1 t t^2 t^3 t^4 t^5;
        0 1 2*t 3*t^2 4*t^3 5*t^4;
        0 0 2 3*t 12*t^2 20*t^3;
    ];

    delta_t = linspace(0, duration, 100);
    
    Bx = [initial_conditions(:,1); final_conditions(:,1)];
    x_coeffs = A \ Bx;
    X = x_coeffs(1) + x_coeffs(2)*delta_t + x_coeffs(3)*delta_t.^2 + x_coeffs(4)*delta_t.^3 + x_coeffs(5)*delta_t.^4 + x_coeffs(6)*delta_t.^5;

    By = [initial_conditions(:,2); final_conditions(:,2)];
    y_coeffs = A \ By;
    Y = y_coeffs(1) + y_coeffs(2)*delta_t + y_coeffs(3)*delta_t.^2 + y_coeffs(4)*delta_t.^3 + y_coeffs(5)*delta_t.^4 + y_coeffs(6)*delta_t.^5;

    Bz = [initial_conditions(:,3); final_conditions(:,3)];
    z_coeffs = A \ Bz;
    Z  = z_coeffs(1) + z_coeffs(2)*delta_t + z_coeffs(3)*delta_t.^2 + z_coeffs(4)*delta_t.^3 + z_coeffs(5)*delta_t.^4 + z_coeffs(6)*delta_t.^5;

    
end