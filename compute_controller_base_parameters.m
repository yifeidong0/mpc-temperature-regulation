function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    a1o = truck.a1o;
    a2o = truck.a2o;
    a3o = truck.a3o;
    a12 = truck.a12;
    a23 = truck.a23;
    m1 = truck.m1;
    m2 = truck.m2;
    m3 = truck.m3;
    InputConstraints = truck.InputConstraints;
    StateConstraints = truck.StateConstraints;
    b_ref = truck.b_ref;
    w = truck.w;
    To = truck.To;
    
    % (2) discretization
    Ts = 60;
    A = eye(3) + Ts .* [-(a1o+a12)/m1, a12/m1, 0; a12/m2, -(a12+a23+a2o)/m2, a23/m2; 0, a23/m3, -(a23+a3o)/m3 ]; 
    B = Ts .* [1/m1, 0; 0, 1/m2; 0, 0];
    B_d = Ts .* [1/m1 0 0; 0 0 1/m2;0 0 1/m3];
    C = eye(3);
    
    % (3) set point computation
    T_sp1 = b_ref(1);
    T_sp2 = b_ref(2);
    T_sp3 = (a23*T_sp2 + a3o*To + w(3))/(a23 + a3o);
    p_sp1 = (a1o+a12)*T_sp1 - a12*T_sp2 - (a1o*To + w(1));
    p_sp2 = -a12*T_sp1 + (a12+a23+a2o)*T_sp2 - a23*T_sp3 - (a2o*To + w(2));
    
    T_sp = [T_sp1; T_sp2; T_sp3];
    p_sp = [p_sp1; p_sp2];
    
    % (4) system constraints
    Pcons = InputConstraints;
    Tcons = StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons - [p_sp, p_sp];
    Xcons = Tcons - [T_sp, T_sp];
    
    % (5) LQR cost function
    Q = 50000 * diag([1, 1, 1]);
    R = eye(2);
    
    % (6) Terminal cost computation via LQR
    [K, P] = dlqr(A, B, Q, R);
    
    % (22) Offset free control
    A_aug = [A B_d; zeros(3), eye(3)];
    B_aug = [B; zeros(3,2)];
    C_aug = [C, zeros(3)];
    D_aug = zeros(3,2);
    %eigen = eig(A_aug - L * C_aug);
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.P = P;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
    param.B_d = B_d;
    param.C = C;
    param.A_aug = A_aug;
    param.B_aug = B_aug;
    param.C_aug = C_aug;
    param.D_aug = D_aug;
    
end