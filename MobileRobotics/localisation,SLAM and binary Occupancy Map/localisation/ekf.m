function [mean_eval,covariance_eval] = ekf(x, y, theta, dt, v,w,landmark_x, landmark_y, state_covariance, measurement_covariance)
    % create variable for computing different coefficients in Kalman filter
    syms x_filter y_filter theta_filter dt_filter v_filter w_filter landmark_x_actual landmark_y_actual
    
    % creating motion equation
    g = [x_filter; y_filter; theta_filter] + [-v_filter/w_filter*sin(theta_filter)+v_filter/w_filter*sin(theta_filter+w_filter*dt_filter);v_filter/w_filter*cos(theta_filter)-v_filter/w_filter*cos(theta_filter+w_filter*dt_filter);w_filter*dt_filter];
    % creating jacobian for motion equation
    G = jacobian(g,[x_filter; y_filter; theta_filter]);
    V = jacobian(g,[v_filter; w_filter]);
    %creating measurement equation
    h = [sqrt((landmark_x_actual-x_filter)^2 + (landmark_y_actual-y_filter)^2); atan2(landmark_y_actual-y_filter, landmark_x_actual-x_filter)];   
    % creating jacobian for measurement equation  
    H = jacobian(h,[x_filter; y_filter; theta_filter]);
    
    % evaluating the jacobian values based on current input
    G_eval = eval(subs(G,[x_filter y_filter theta_filter dt_filter v_filter w_filter], [x y theta dt v w]));
    V_eval = eval(subs(V,[x_filter y_filter theta_filter dt_filter v_filter w_filter], [x y theta dt v w]));

    %Prediction Step
    % calculating the mean based on present value
    mean_eval = eval(subs(g,[x_filter y_filter theta_filter dt_filter v_filter w_filter], [x y theta dt v w]));
    %calculating covariance
    covariance_eval = G_eval*state_covariance*G_eval' + V_eval*measurement_covariance*V_eval';
    
    %update Step
    % calculating measurement difference between original landmark location
    % and the location computed in previous iteration
    measurement_diff = [x; y] - eval(subs(h,[landmark_x_actual landmark_y_actual x_filter y_filter], [landmark_x landmark_y mean_eval(1) mean_eval(2)]));
    H_eval = eval(subs(H,[landmark_x_actual landmark_y_actual x_filter y_filter], [landmark_x landmark_y mean_eval(1) mean_eval(2)]));
    s_t = H_eval*covariance_eval*H_eval' + eye(2);
    k_t = covariance_eval*H_eval'/(s_t); % kalman gain
    mean_eval = mean_eval + k_t*measurement_diff; % updating the mean
    covariance_eval = covariance_eval - k_t*s_t*k_t'; % updating the covariance
end

