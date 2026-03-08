function [Q,Theta,axis,delta_t,theta_coeffs] = cubicAngularTrajectory(initialOrientation,initialAngularVelocity,finalOrientation,finalAngularVelocity,duration,dt)
    % function to create a trajectory of frame rotation
    
    % INTPUTS : 
    % initialOrientation as a quaternion,
    % initialAngularVelocity as a double in rad/s,
    % finalOrientation as a quaternion,
    % finalAngularVelocity as a double in rad/s,
    % duration as a double in s

    % OUTPUT :
    % Array of quaternions for trajectory
    % Theta trajectory
    % axis 1x3
    % delta_t timed array
    % theta_coeffs, coefficients of the cubic function

    arguments (Input)
        initialOrientation (1,1) quaternion
        initialAngularVelocity (1,1)
        finalOrientation (1,1) quaternion
        finalAngularVelocity (1,1)
        duration (1,1)
        dt (1,1) double = 0.1
    end

    arguments (Output)
        Q
        Theta
        axis
        delta_t
        theta_coeffs
    end

    delta_q = initialOrientation*finalOrientation.conj();
    delta_rotvec = delta_q.rotvec();
    delta_theta = norm(delta_rotvec);

    t = duration;

    initial_state = [0;initialAngularVelocity];
    final_state = [delta_theta;finalAngularVelocity];
    b=[initial_state;final_state];

    A = [
        1 0 0 0;
        0 1 0 0;
        1 t t^2 t^3;
        0 1 2*t 3*t^2;
    ];

    theta_coeffs = A\b;
    delta_t = 0:dt:duration;

    Theta = theta_coeffs(1) + theta_coeffs(2)*delta_t + theta_coeffs(3)*delta_t.^2 + theta_coeffs(4)*delta_t.^3;
    if delta_theta < 1e-12
        axis = [1 0 0];
    else
        axis = delta_rotvec/delta_theta;
    end
    
    Q = zeros(size(delta_t,2),1,"quaternion");
  
    for i = 1:size(Theta',1)
        delta_qi = quaternion(Theta(i)*axis,'rotvec');

        q = delta_qi*initialOrientation;
        Q(i) = q;
    end
end