function model = wheelc_push_model()

import casadi.*

%% system dimensions
%state and control dimension
nx = 6;
nu = 6;
%np = 12;
np=3;%Fix
%% sample time
%dt = 0.01;


%% named symbolic variables  
%State
rg_x = SX.sym('rg_x'); % x cog position
rg_y = SX.sym('rg_y'); % y cog position
fh_r_x = SX.sym('fh_r_x'); % x right hand force
fh_r_y = SX.sym('fh_r_y'); % y right hand force
fh_l_x = SX.sym('fh_l_x'); % x left hand force
fh_l_y = SX.sym('fh_l_y'); % y left hand force

%Compact state variables
rg = vertcat(rg_x, rg_y);
fh = vertcat(fh_r_x, fh_r_y, fh_l_x, fh_l_y);

%Control inputs
rg_x_dot = SX.sym('rg_x_dot'); % x cog velocity
rg_y_dot = SX.sym('rg_y_dot'); % y cog velocity
fh_r_x_dot = SX.sym('fh_r_x_dot'); % x right hand force derivative
fh_r_y_dot = SX.sym('fh_r_y_dot'); % y right hand force derivative
fh_l_x_dot = SX.sym('fh_l_x_dot'); % x left hand force derivative
fh_l_y_dot = SX.sym('fh_l_y_dot'); % y left hand force derivative

%Compact control variables
rg_dot = vertcat(rg_x_dot, rg_y_dot);
fh_dot = vertcat(fh_r_x_dot, fh_r_y_dot, fh_l_x_dot, fh_l_y_dot);

%% (unnamed) symbolic variables
sym_x = vertcat(rg, fh);
sym_u = vertcat(rg_dot, fh_dot);
sym_p = SX.sym('p', np, 1);
sym_xdot = sym_u;


%% system parameters
g = 9.81;
M = sym_p(1); %Robot mass
zh_r = sym_p(2); %Z coordinate of right hand in pelvis frame
zh_l = sym_p(3); %Z coordinate of left hand in pelvis frame

%% Constraints
zmp_constraint = (M*rg*g - zh_r*fh(1:2) - zh_l*fh(3:4))/(M*g);
expr_h = vertcat(zmp_constraint);

%% Dynamics
expr_f_expl = sym_xdot;
expr_f_impl = expr_f_expl - sym_xdot;

%% populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_p = sym_p;
model.sym_u = sym_u;
model.sym_xdot = sym_xdot;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
