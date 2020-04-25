% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    %% Here you need to implement the X_LQR computation and assign the result.
    A = param.A;
    B = param.B;
    Q = param.Q;
    R = param.R;
    K = -dlqr(A,B,Q,R);
    systemLQR = LTISystem('A', A + B * K);
    Xp = Polyhedron('A',[1,0,0; 0,1,0; 0,-1,0; K; -K], 'b', [param.Xcons(1,2);param.Xcons(2,2); -param.Xcons(2,1);param.Ucons(:,2);-param.Ucons(:,1)]);
%     figure(2)
%     Xp.plot(), alpha(0.25)
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    InvSetLQR = systemLQR.invariantSet();
    A_x = InvSetLQR.A;
    b_x = InvSetLQR.b;
%     figure(1)
%     hold on
%     InvSetLQR.plot(), alpha(0.25),xlabel('x_1'), ylabel('x_2'),zlabel('x_3')
end

