function obj = prediction(obj, motion)
% returns state predicttion based on IMU measurement.
% state: 5x5 matrix,  [rotation, velocity, position]
% motion: contains omega, and acceleration, 3x2 matrix?

    state(1) = obj.mu(1,3);
    state(2) = obj.mu(2,3);
    state(3) = atan2(obj.mu(2,1), obj.mu(1,1));
    H_prev = obj.posemat(state);
    state_pred = obj.gfun(state, u);
    H_pred = obj.posemat(state_pred);

    u_se2 = logm(H_prev \ H_pred);

    Adjoint = @(X) [X(1:2,1:2), [X(2,3); -X(1,3)]; 0 0 1];
    AdjX = Adjoint(H_prev);

    obj.propagation(u_se2, AdjX);

end