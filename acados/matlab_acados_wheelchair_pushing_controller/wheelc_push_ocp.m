clear mex, clear all,  close all, clc
check_acados_requirements()

%% Parameters  
% (all except lb and ub will be updated at each step from 
% C++ code, so their value in this script is not relevant)
M = 75;
zh_r = 0.5;
zh_l = 0.5;
parameters=[M; zh_r; zh_l];

%Initial conditions
rg_x_0 = 0; % x cog position
rg_y_0 = 0; % y cog position
fh_r_x_0 = 0; % x right hand force
fh_r_y_0 = 0; % y right hand force
fh_l_x_0 = 0; % x left hand force
fh_l_y_0 = 0; % y left hand force


x0 = [rg_x_0;rg_y_0;fh_r_x_0;fh_r_y_0;...
        fh_l_x_0;fh_l_y_0];

%State lower bound (lb) and upper bound (ub)
rg_x_lb = -10; rg_x_ub = 10;
rg_y_lb = -10; rg_y_ub = 10;
fh_r_x_lb = -100; fh_r_x_ub = 100;
fh_r_y_lb = -100; fh_r_y_ub = 100;
fh_l_x_lb = -100; fh_l_x_ub = 100;
fh_l_y_lb = -100; fh_l_y_ub = 100;

%Control input lower bound (lb) and upper bound (ub)
rg_x_dot_lb = -10; rg_x_dot_ub = 10;
rg_y_dot_lb = -10; rg_y_dot_ub = 10;
fh_r_x_dot_lb = -100; fh_r_x_dot_ub = 100;
fh_r_y_dot_lb = -100; fh_r_y_dot_ub = 100;
fh_l_x_dot_lb = -100; fh_l_x_dot_ub = 100;
fh_l_y_dot_lb = -100; fh_l_y_dot_ub = 100;
%xi_lb = 0; xi_ub = 100;

%Constr lower bound (lb) and upper bound (ub)
zmp_x_lb = -100; zmp_x_ub = 100; %This has to be updated at each timestep from C code
zmp_y_lb = -100; zmp_y_ub = 100;


%State weights
w_rg_x = 1;
w_rg_y = 1;
w_fh_r_x = 1;
w_fh_r_y = 1; 
w_fh_l_x = 1; 
w_fh_l_y = 1;


%Control weights
w_rg_x_dot = 1; 
w_rg_y_dot = 1; 
w_fh_r_x_dot = 1; 
w_fh_r_y_dot = 1; 
w_fh_l_x_dot = 1; 
w_fh_l_y_dot = 1;


%Constraints weights
w_terminal = 0.1;




%% optionsocp_nlp_solver
ocp_N = 1;
compile_interface = 'auto';
codgen_model = 'true';


% ocp
%ocp_nlp_solver = 'sqp';
ocp_nlp_solver = 'sqp_rti';
ocp_nlp_solver_max_iter = 5;
ocp_qp_solver = 'partial_condensing_hpipm';
%ocp_qp_solver = 'full_condensing_hpipm';
ocp_qp_solver_cond_N = 1;
ocp_sim_method = 'erk';
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 1;

ocp_cost_type = 'linear_ls';
%ocp_cost_type = 'nonlinear_ls';
%ocp_cost_type = 'ext_cost';
ocp_levenberg_marquardt = 1e-1;
qp_solver_warm_start = 1;
qp_solver_iter_max = 100; % default is 50; OSQP needs a lot sometimes.
%% model
model = wheelc_push_model();


%% setup problem
%N = 25;
%T = 30; % time horizon length
dt = 0.01;
nx = model.nx;
nu = model.nu;
ny = nx+nu;
ny_e = nx; % number of outputs in mayer term
nbx = nx; % number of state bounds
nbu = nu; % number of input bounds

%cost
Vx = zeros(ny, nx); for ii=1:nx Vx(ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vu = zeros(ny, nu); for ii=1:nu Vu(nx+ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
Q = diag([w_rg_x,w_rg_y,w_fh_r_x,w_fh_r_y,w_fh_l_x,w_fh_l_y]);
R = diag([w_rg_x_dot,w_rg_y_dot,w_fh_r_x_dot,w_fh_r_y_dot,w_fh_l_x_dot,w_fh_l_y_dot]);
W = blkdiag(Q, R); % weight matrix in lagrange term
W_e = w_terminal*Q; % weight matrix in mayer term
%This will be updated at each step
yref = zeros(ny, 1); % output reference in lagrange term
yref_e = zeros(ny_e, 1); % output reference in mayer term


%constraints
Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [rg_x_lb;rg_y_lb;fh_r_x_lb;fh_r_y_lb;fh_l_x_lb;fh_l_y_lb];
ubx = [rg_x_ub;rg_y_ub;fh_r_x_ub;fh_r_y_ub;fh_l_x_ub;fh_l_y_ub];


Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [rg_x_dot_lb;rg_y_dot_lb;fh_r_x_dot_lb;fh_r_y_dot_lb;fh_l_x_dot_lb;fh_l_y_dot_lb];
ubu = [rg_x_dot_ub;rg_y_dot_ub;fh_r_x_dot_ub;fh_r_y_dot_ub;fh_l_x_dot_ub;fh_l_y_dot_ub];

%H_min = [lambda_m_lb;lambda_p_lb;contact_constr_lb;phi_dot_rate_constr_lb;phi_dot_rate_constr_lb];
%H_max = [lambda_m_ub;lambda_p_ub;contact_constr_ub;phi_dot_rate_constr_ub;phi_dot_rate_constr_ub];
H_min = [zmp_x_lb; zmp_y_lb];
H_max = [zmp_x_ub; zmp_y_ub];

%% acados ocp model
ocp_model = acados_ocp_model();
model_name = 'wheelc_push';
ocp_model.set('name', model_name);
ocp_model.set('T', ocp_N*dt);

% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_p', model.sym_p);

% cost
ocp_model.set('cost_type', ocp_cost_type);
ocp_model.set('cost_type_e', ocp_cost_type);
ocp_model.set('cost_Vu', Vu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);
ocp_model.set('cost_y_ref', yref);
ocp_model.set('cost_y_ref_e', yref_e);

% dynamics
ocp_model.set('dyn_type', 'explicit');
ocp_model.set('dyn_expr_f', model.expr_f_expl);

% constraints
ocp_model.set('constr_x0', x0);
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);
ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_lh', H_min); % lower bound on h
ocp_model.set('constr_uh', H_max);  % upper bound on h
%ocp_model.set('constr_type', 'auto');

ocp_model.model_struct;

%% acados ocp opts

ocp_opts = acados_ocp_opts();
ocp_opts.set('parameter_values',parameters);
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', ocp_N);
ocp_opts.set('nlp_solver', ocp_nlp_solver);
ocp_opts.set('nlp_solver_max_iter', ocp_nlp_solver_max_iter);
ocp_opts.set('qp_solver', ocp_qp_solver);
if (strcmp(ocp_qp_solver, 'partial_condensing_hpipm'))
    ocp_opts.set('qp_solver_cond_N', ocp_qp_solver_cond_N); %New horizon after partial condensing
end
ocp_opts.set('levenberg_marquardt', ocp_levenberg_marquardt);
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
ocp_opts.set('regularize_method', 'convexify');
ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);

ocp_opts.opts_struct;

%% acados ocp
ocp = acados_ocp(ocp_model, ocp_opts);
ocp_model.set('name', model_name);


%reference = importdata("eight.txt");
% x_traj_init = zeros(nx, ocp_N+1);
x_traj_init = repmat(x0, 1, ocp_N+1);
u_traj_init = zeros(nu, ocp_N);


% set trajectory initialization
ocp.set('init_x', x_traj_init);
ocp.set('init_u', u_traj_init);

%cost_val_ocp = zeros(n_sim, 1);

ocp.set('constr_x0', x0);
    
% solve OCP
ocp.solve();

% get cost value
%cost_val_ocp(ii) = ocp.get_cost();

% get solution
x_traj = ocp.get('x');
u_traj = ocp.get('u');


status = ocp.get('status');

if status==0
    %fprintf('\nsuccess!\n\n');
else
    fprintf('\nsolution failed with status %d!\n\n', status);
    %break;
end



%{
%% Plots
%ts = linspace(0, T, N+1);
States = {'xp';'yp';'theta';'phi';...
    'f_x';'f_y';'xd';'yd'};%;'kv_x';'kv_y'};
figure; hold on;
for i=1:4
    subplot(4, 1, i);
    plot(x_traj(i,:)); grid on;
    ylabel(States{i});
    xlabel('t [s]')
    hold on
    subplot(4, 1, i);
    %plot(reference(i,1:n_sim))
end

figure; hold on;
for i=5:8
    subplot(4, 1, i-4);
    plot(x_traj(i,:)); grid on;
    ylabel(States{i});
    xlabel('t [s]')
end

%ts = linspace(0, T, N);
figure; hold on;
ctrl = {'phi_p_dot', 'phi_m_dot', 'xd_dot', 'yd_dot', 'epsilon'};
for i=1:5
    subplot(5, 1, i);
    plot(u_traj(i,:)); grid on;
    ylabel(ctrl{i});
    xlabel('t [s]')
end
%}

%% go embedded
% to generate templated C code
ocp.generate_c_code;
