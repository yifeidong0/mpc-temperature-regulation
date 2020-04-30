% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param  yalmip_optimizer  x_es  d_es  p_last

if isempty(param)
    % initialize controller, if not done already
    [param, yalmip_optimizer] = init();
    x_es = [3.1 1.1 0.1]';
    d_es = [0.01 -0.01 0.01]';
else
    % update estimate state and disturbance
    result = param.A_aug * [x_es; d_es] + param.B_aug * (p_last - param.p_sp) ...
        + param.L * ((T - param.T_sp) - param.C_aug * [x_es; d_es]);
    x_es = result(1:3);
    d_es = result(4:6);
end

% compute steady state inputs and outputs
r = zeros(3,1);
result_sp = [(param.A-eye(3)), param.B; param.C, zeros(3,2)] ...
    \ [-param.B_d * d_es; r];
x_s = result_sp(1:3);
u_s = result_sp(4:5);

% evaluate control action by solving MPC problem, e.g.
input = [(x_es - x_s); d_es];
[u_mpc,errorcode] = yalmip_optimizer(input);
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc + u_s + param.p_sp;
p_last = p;

end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters
[param.K, param.P] = dlqr(param.A, param.B, param.Q, param.R);
param.L = 0.05 * ones(6,3); % cannot be too large

% implement your MPC using Yalmip here
N = 30;
nx = size(param.A,1);
nu = size(param.B,2);

U = sdpvar(repmat(nu,1,N-1), ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N), ones(1,N),'full');
d = sdpvar(repmat(nx,1,N), ones(1,N),'full');

Ax=[eye(3);-eye(3)];
bx=[param.Xcons(:,2); -param.Xcons(:,1)];
Au=[eye(2);-eye(2)];
bu=[param.Ucons(:,2); -param.Ucons(:,1)];
[Ax_t, bx_t] = compute_X_LQR;

objective = 0;
constraints = [];
for i = 1:N-1
    constraints = [constraints, X{i+1} == param.A * X{i} + param.B * U{i} + param.B_d * d{i}];
    constraints = [constraints, d{i+1} == d{i}];  
    constraints = [constraints, Ax * X{i+1} <= bx];
    constraints = [constraints, Au * U{i} <= bu];
    objective = objective + X{i}' * param.Q * X{i} + U{i}' * param.R * U{i} ;
end
constraints = [constraints, Ax_t * X{end} <= bx_t];
objective = objective + X{end}' * param.P * X{end};
% Initial condition
x0 = sdpvar(6,1);
constraints = [constraints, X{1} == x0(1:3), d{1} == x0(4:6)];

ops = sdpsettings('verbose',0,'solver','quadprog');
fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = optimizer(constraints,objective,ops,x0,U{1} );
end