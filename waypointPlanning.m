function [translation_trajectory,rotation_trajectory,linear_velocity,angular_velocity,timesteps] = waypointPlanning(waypoint_positions,waypoint_orientations,via_point_velocity)
%waypointPlanning generate cubic polynomialtrajectories with via points
%   plans ideal via point velocities and returns a complete trajectory
%   between the first and finish waypoint, assumes 0 angular velocities at
%   the via points
    arguments (Input)
        waypoint_positions % nx3 matrix of positions
        waypoint_orientations quaternion % nx1 quaternion array, n should be the same as the waypoint_positions
        via_point_velocity % (n-2,1) array of velocities at via point, in ms-1
    end
    
    arguments (Output)
        translation_trajectory % mx3 array of positions
        rotation_trajectory % mx1 array of quaternions
        linear_velocity % mx3 array
        angular_velocity % mx3 array
        timesteps % mx1 array of timing
    end
    
    % between every two points, need to find out the ideal velocity vector and
    % duration. How tho?
    % velocity vector at a via point can be along the vector joining the positions around
    % it
    % magnitude? it can be hardcoded for now
    % duration between motions? can be limited to have certain end effector
    % accelerations

    via_positions = waypoint_positions(2:end-1,:);

    % now intermediate velocities
    num_via_points = size(via_positions,1);
    intermediate_velocity_vectors = zeros(num_via_points,3);
    for i = 2:num_via_points+1
        intermediate_velocity_unit_vector = (waypoint_positions(i+1,:) - waypoint_positions(i-1,:))/norm(waypoint_positions(i+1,:) - waypoint_positions(i-1,:),2);
        intermediate_velocity_vectors(i-1, :) = intermediate_velocity_unit_vector * via_point_velocity(i-1);
    end
    
    intermediate_velocity_vectors = [0 0 0;intermediate_velocity_vectors;0 0 0];
    
    % now gotta generate trajectory, with a heuristic for duration
    % via_points = wayopint-2
    max_vel = 0.5;
    translation_trajectory = [];
    rotation_trajectory = quaternion.empty(0,1);
    linear_velocity = [];
    angular_velocity = [];
    timesteps = [];

    for i=1:(size(waypoint_positions,1)-1)
        dt = norm(waypoint_positions(i,:)-waypoint_positions(i+1,:))/max_vel;

        % get translation trajectory
        initial_conditions = [waypoint_positions(i,:);intermediate_velocity_vectors(i,:)];
        final_conditions = [waypoint_positions(i+1,:);intermediate_velocity_vectors(i+1,:)];

        % get linear velocity
        [dX,dY,dZ,dtimesteps,dx_coeffs,dy_coeffs,dz_coeffs] = cubicPolynomialTrajectory(initial_conditions,final_conditions,dt,0.01);

        dVx = dx_coeffs(2) + 2*dx_coeffs(3)*dtimesteps + 3*dx_coeffs(4)*dtimesteps.^2;
        dVy = dy_coeffs(2) + 2*dy_coeffs(3)*dtimesteps + 3*dy_coeffs(4)*dtimesteps.^2;
        dVz = dz_coeffs(2) + 2*dz_coeffs(3)*dtimesteps + 3*dz_coeffs(4)*dtimesteps.^2;

        % now angular trajectory and velocity
        [dQ,~,daxis,dtimesteps,dtheta_coeffs] = cubicAngularTrajectory(waypoint_orientations(i),0.0,waypoint_orientations(i+1),0.0,dt,0.01);
        omega = dtheta_coeffs(2) + 2*dtheta_coeffs(3)*dtimesteps + 3*dtheta_coeffs(4)*dtimesteps.^2;
        dangular_velocity = omega' .* daxis;

        translation_trajectory = [translation_trajectory; dX' dY' dZ'];
        rotation_trajectory = [rotation_trajectory; dQ];
        linear_velocity = [linear_velocity; dVx' dVy' dVz'];
        angular_velocity = [angular_velocity; dangular_velocity];
        
        if isempty(timesteps)
            timesteps = dtimesteps';
        else
            timesteps = [timesteps; timesteps(end) + dtimesteps'];
        end

    end


end