clc; clear; close all;

%% --- Harita secimi ---
choice = menu('Map sec', 'Circle', 'Rectangle', 'Hand Figure');
switch choice
    case 1, run('/Users/kasimesen/Desktop/Tasarım/2. Week/YuvarlakMap.m');
    case 2, run('/Users/kasimesen/Desktop/Tasarım/2. Week/KareMap.m');
    case 3, run('/Users/kasimesen/Desktop/Tasarım/2. Week/HandFigureMap.m');
    otherwise, error('Secim yapilmadi.');
end
if ~exist('waypoints','var') || size(waypoints,2)~=2 || size(waypoints,1)<2
    error('waypoints bulunamadi veya format yanlis. Nx2 olmali.');
end

%% --- Start / Goal ---
if exist('startPt','var') && numel(startPt)==2
    sp = startPt(:).';
else
    sp = waypoints(1,:);
end
if exist('goal','var') && numel(goal)==2
    gp = goal(:).';
else
    gp = waypoints(end,:);
end

%% --- Referans yol ve eğrilik ---
wp = waypoints;
wp = wp([true; sqrt(sum(diff(wp).^2,2)) > 1e-9],:);
s_raw  = [0; cumsum(sqrt(sum(diff(wp).^2,2)))];
[sU,ia] = unique(s_raw,'stable');
wpU = wp(ia,:);

ds_target = 0.1;
N_dense   = max(round(sU(end)/ds_target), 1500);
s_dense   = linspace(0, sU(end), N_dense)';
x_ref     = interp1(sU, wpU(:,1), s_dense, 'pchip');
y_ref     = interp1(sU, wpU(:,2), s_dense, 'pchip');

% GERCEK ds
ds_real = s_dense(2) - s_dense(1);

xp  = gradient(x_ref, s_dense);   yp  = gradient(y_ref, s_dense);
xpp = gradient(xp,    s_dense);   ypp = gradient(yp,    s_dense);
kappa_ref = (xp.*ypp - yp.*xpp) ./ (xp.^2 + yp.^2).^1.5;
theta_ref = atan2(yp, xp);

[~,idx0] = min((x_ref - sp(1)).^2 + (y_ref - sp(2)).^2);
theta0   = theta_ref(idx0);

%% --- Araç parametreleri ---
L         = 2.7;
lr        = 1.35;
lf        = L - lr;
v_const   = 5;
m         = 1500;
Iz        = 2250;
Cf        = 2*19000;
Cr        = 2*19000;
delta_max = deg2rad(30);
ddelta_max_per_step = 10 * 0.02;

%% --- Sim parametreleri ---
Ts       = 0.02;
T_end    = 350;
goal_tol = 1.0;
n_laps = 3;

%% --- MPC ayarlari ---
N_p = 60;
Q   = diag([70, 10, 50, 30]);
R   = 15.0;
nx  = 4;
nu  = 1;
%% --- Hata modeli ---
Ac = [0, 1, 0, 0;
      0, -(Cf+Cr)/(m*v_const), (Cf+Cr)/m, (-Cf*lf+Cr*lr)/(m*v_const);
      0, 0, 0, 1;
      0, -(Cf*lf-Cr*lr)/(Iz*v_const), (Cf*lf-Cr*lr)/Iz, -(Cf*lf^2+Cr*lr^2)/(Iz*v_const)];
Bc = [0;
      Cf/m;
      0;
      Cf*lf/Iz];
Ec = [0;
      -(Cf*lf-Cr*lr)/m - v_const^2;
      0;
      -(Cf*lf^2+Cr*lr^2)/Iz];

%% --- Euler discretization ---
Ad = eye(nx) + Ts * Ac;
Bd = Ts * Bc;
Ed = Ts * Ec;

%% --- Terminal cost ---
[~, P_term] = dlqr(Ad, Bd, Q, R);

%% --- Yığın matrisleri ---
Phi   = zeros(nx*N_p, nx);
Gamma = zeros(nx*N_p, nu*N_p);
Psi   = zeros(nx*N_p, N_p);

for i = 1:N_p
    Phi((i-1)*nx+1:i*nx, :) = Ad^i;
    for j = 1:i
        Aij = Ad^(i-j);
        Gamma((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Aij * Bd;
        Psi((i-1)*nx+1:i*nx, j) = Aij * Ed;
    end
end

%% --- Ağırlık ---
Qbar = kron(eye(N_p), Q);
Qbar(end-nx+1:end, end-nx+1:end) = P_term;
Rbar = kron(eye(N_p), R);

%% --- H matrisi ---
H_mpc = 2*(Gamma'*Qbar*Gamma + Rbar);
H_mpc = (H_mpc + H_mpc')/2;

%% --- Rate-limit kısıt---
D_diff = eye(N_p) - diag(ones(N_p-1,1), -1);
A_ineq_const = [ D_diff;
                -D_diff];

mapScale = min(max(x_ref)-min(x_ref), max(y_ref)-min(y_ref));
approach_dist = 0.08 * mapScale;

%% --- Workspace ---
mpc_params.N_p        = N_p;
mpc_params.nx         = nx;
mpc_params.nu         = nu;
mpc_params.L          = L;
mpc_params.v_const    = v_const;
mpc_params.delta_max  = delta_max;
mpc_params.ddelta_max = ddelta_max_per_step;
mpc_params.ds_real    = ds_real;
mpc_params.Phi   = Phi;
mpc_params.Gamma = Gamma;
mpc_params.Psi   = Psi;
mpc_params.Qbar  = Qbar;
mpc_params.H     = H_mpc;
mpc_params.A_ineq_const = A_ineq_const;

ref_path.x     = x_ref;
ref_path.y     = y_ref;
ref_path.theta = theta_ref;
ref_path.kappa = kappa_ref;
ref_path.s     = s_dense;
ref_path.N     = numel(s_dense);

sim_params.Ts            = Ts;
sim_params.T_end         = T_end;
sim_params.goal_tol      = goal_tol;
sim_params.approach_dist = approach_dist;
sim_params.startPt       = sp;
sim_params.goal          = gp;
sim_params.theta0        = theta0;

assignin('base','mpc_params', mpc_params);
assignin('base','ref_path',   ref_path);
assignin('base','sim_params', sim_params);
assignin('base','x_ref',      x_ref);
assignin('base','y_ref',      y_ref);
assignin('base','theta_ref',  theta_ref);
assignin('base','kappa_ref',  kappa_ref);
assignin('base','s_dense',    s_dense);

assignin('base','L',         L);
assignin('base','lr',        lr);
assignin('base','v_const',   v_const);
assignin('base','Ts',        Ts);
assignin('base','T_end',     T_end);
assignin('base','x0_init',   sp(1));
assignin('base','y0_init',   sp(2));
assignin('base','theta0',    theta0);


%% MPC_Controller

assignin('base','ref_x',         x_ref);
assignin('base','ref_y',         y_ref);
assignin('base','ref_th',        theta_ref);
assignin('base','ref_kp',        kappa_ref);
assignin('base','Phi',           Phi);
assignin('base','Gamma',         Gamma);
assignin('base','Psi',           Psi);
assignin('base','H_mpc',         H_mpc);
assignin('base','A_in',          A_ineq_const);
assignin('base','Qbar',          Qbar);
assignin('base','N_p',           N_p);
assignin('base','v_const',       v_const);
assignin('base','delta_max',     delta_max);
assignin('base','ddelta_max',    ddelta_max_per_step);
assignin('base','approach_dist', approach_dist);
assignin('base','ds_real',       ds_real);
assignin('base','goal_x',   gp(1));
assignin('base','goal_y',   gp(2));
assignin('base','goal_tol', goal_tol);
assignin('base','n_laps',   n_laps);

fprintf('Parametreler workspace''e yuklendi.\n');

fprintf('\n=== MPC Setup tamamlandi ===\n');
fprintf('Harita: %d nokta, ds_real = %.4f m\n', ref_path.N, ds_real);
fprintf('MPC: N_p=%d, Q=diag([%g,%g]), R=%g\n', N_p, Q(1,1), Q(2,2), R);
fprintf('Rate limit: %.4f rad/adim (%.2f deg)\n', ...
        ddelta_max_per_step, rad2deg(ddelta_max_per_step));

%% --- Onizleme ---
figure('Color','w','Name','Reference path preview');
subplot(1,2,1);
plot(x_ref, y_ref, 'b-', 'LineWidth', 1.5); hold on;
plot(sp(1), sp(2), 'go', 'MarkerSize', 9, 'LineWidth', 2);
plot(gp(1), gp(2), 'rx', 'MarkerSize', 11, 'LineWidth', 2);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]'); title('Referans yol');
legend('Yol','Start','Goal','Location','best');

subplot(1,2,2);
plot(s_dense, kappa_ref, 'r-', 'LineWidth', 1.2); grid on;
xlabel('s [m]'); ylabel('\kappa [1/m]'); title('Egrilik profili');