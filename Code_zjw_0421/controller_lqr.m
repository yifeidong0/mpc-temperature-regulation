% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

%% compute control action
% K = lqr(param.A, param.B, param.Q, param.R);
% alpha = diag([0.5, 0.2]); % this is a discount on K, which is important in ... 
% % ... making T1 converge faster(to meet the T(30) constraint) and T2 slower(to meet the state constraint)
% delta_u_temp = - K * (T - param.T_sp); % delta_u == -K * delta_x
% delta_u = zeros(2,1);
% % input constraints
% for i = 1:2
%     if delta_u_temp(i) > param.Ucons(i,2)
%         delta_u(i) = param.Ucons(i,2);
%     elseif delta_u_temp(i) < param.Ucons(i,1)
%         delta_u(i) = param.Ucons(i,1);
%     else
%         delta_u(i) = delta_u_temp(i);
%     end
% end
p = param.p_sp - param.K*(T - param.T_sp); % For 1*1 case, alpha = 0.24, with the T(30) constraints not satisfied.

end

function param = init()
param = compute_controller_base_parameters;
 [param.K, param.P] = dlqr(param.A, param.B, param.Q, param.R);
% add additional parameters if necessary, e.g.
% param.P = ;
% param.F = ;
end
