function [X,Y,Z,delta_t,x_coeffs,y_coeffs,z_coeffs] = cubicPolynomialTrajectory(initial_conditions,final_conditions,duration,dt)
    %cubicPolynomialTrajectory function returns X,Y,Z delta_t and
    %coefficients
    %   intial_condition and final : initial_position;initial_velocity,final_position,final_velocity, 4x3 matrix
    %   duration is a float
    
    arguments (Input)
        initial_conditions
        final_conditions
        duration
        dt (1,1) double = 0.1
    end

    arguments (Output)
        X
        Y
        Z
        delta_t
        x_coeffs
        y_coeffs
        z_coeffs
    end
    
    t = duration;
    A = [
        1 0 0 0 ;
        0 1 0 0 ;
        1 t t^2 t^3;
        0 1 2*t 3*t^2;
    ];

    
    delta_t = 0:dt:duration;

    Bx = [initial_conditions(:,1); final_conditions(:,1)];
    x_coeffs = A\Bx;
    X = x_coeffs(1) + x_coeffs(2)*delta_t + x_coeffs(3)*delta_t.^2 + x_coeffs(4)*delta_t.^3;

    By = [initial_conditions(:,2); final_conditions(:,2)];
    y_coeffs = A\By;
    Y = y_coeffs(1) + y_coeffs(2)*delta_t + y_coeffs(3)*delta_t.^2 + y_coeffs(4)*delta_t.^3;

    Bz = [initial_conditions(:,3); final_conditions(:,3)];
    z_coeffs = A\Bz;
    Z  = z_coeffs(1) + z_coeffs(2)*delta_t + z_coeffs(3)*delta_t.^2 + z_coeffs(4)*delta_t.^3;

end