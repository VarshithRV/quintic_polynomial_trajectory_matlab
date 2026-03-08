function [X,Y,Z,t,timesteps,x_coeffs,y_coeffs,z_coeffs] = safeCubicPolynomialTrajectory(initial_conditions,final_conditions,max_vel)
    %cubicPolynomialTrajectory function returns X,Y,Z delta_t and
    %coefficients
    %   intial_condition and final : initial_position;initial_velocity,final_position,final_velocity, 4x3 matrix
    %   duration is a float
    %   max_vel is the velocity limits in
    %   task space in SI units, ms-1, must be positive always
    
    arguments (Input)
        initial_conditions
        final_conditions
        max_vel
    end

    arguments (Output)
        X
        Y
        Z
        t
        timesteps
        x_coeffs
        y_coeffs
        z_coeffs
    end
    
    % % need to calculate duration
    % t = 0.01;% min timestep
    % current_max_vel = 99;
    
    % while(current_max_vel>max_vel)
    %     A = [
    %         1 0 0 0 ;
    %         0 1 0 0 ;
    %         1 t t^2 t^3;
    %         0 1 2*t 3*t^2;
    %     ];
    % 
    %     Bx = [initial_conditions(:,1); final_conditions(:,1)];
    %     x_coeffs = A\Bx;
    %     By = [initial_conditions(:,2); final_conditions(:,2)];
    %     y_coeffs = A\By;
    %     Bz = [initial_conditions(:,3); final_conditions(:,3)];
    %     z_coeffs = A\Bz;
    % 
    %     ts = 0:0.1:t;
    % 
    %     Vx = x_coeffs(2) + 2*x_coeffs(3)*ts + 3*x_coeffs(4)*ts.^2;
    %     Vy = y_coeffs(2) + 2*y_coeffs(3)*ts + 3*y_coeffs(4)*ts.^2;
    %     Vz = z_coeffs(2) + 2*z_coeffs(3)*ts + 3*z_coeffs(4)*ts.^2;
    % 
    %     speed = sqrt(Vx.^2 + Vy.^2 + Vz.^2);
    %     current_max_vel = max(speed);
    % 
    %     t = t+0.01;
    % end
    A = [
            1 0 0 0 ;
            0 1 0 0 ;
            1 t t^2 t^3;
            0 1 2*t 3*t^2;
        ];
    
    Bx = [initial_conditions(:,1); final_conditions(:,1)];
    x_coeffs = A\Bx;
    By = [initial_conditions(:,2); final_conditions(:,2)];
    y_coeffs = A\By;
    Bz = [initial_conditions(:,3); final_conditions(:,3)];
    z_coeffs = A\Bz;

    timesteps = 0:0.1:t;
    X = x_coeffs(1) + x_coeffs(2)*timesteps+ x_coeffs(3)*timesteps.^2 + x_coeffs(4)*timesteps.^3;
    Y = y_coeffs(1) + y_coeffs(2)*timesteps+ y_coeffs(3)*timesteps.^2 + y_coeffs(4)*timesteps.^3;
    Z  = z_coeffs(1) + z_coeffs(2)*timesteps+ z_coeffs(3)*timesteps.^2 + z_coeffs(4)*timesteps.^3;

end