function [delta_cmd, ey, eh, idx_out, lap_out, done_out, slack_out, exitflag_out] = mpc_solver( ...
    x, y, th, vy, yaw_rate, u_prev, ...
    x_ref, y_ref, theta_ref, kappa_ref, ...
    idx_prev_in, lap_prev, done_prev, ...
    Phi, Gamma, Psi, H_mpc, A_ineq_const, ...
    N_p, v_const, Ts, ...
    delta_max, ddelta_max, approach_dist, ...
    Qbar, ds_real, ...
    goal_x, goal_y, goal_tol, n_laps_target, ...
    alpha_max, slack_w_lin, slack_w_quad, L, lr, kappa_lim)
    coder.extrinsic('quadprog','optimoptions');

    nRef = numel(x_ref);

    %% Çıkışlar
    delta_cmd = double(u_prev);
    ey        = 0;
    eh        = 0;
    idx_out   = double(idx_prev_in);
    lap_out   = double(lap_prev);
    done_out  = double(done_prev);
    slack_out    = 0;      % kayma acisi kisiti gevsemesi s [rad] (tani)
    exitflag_out = 0;      % quadprog exitflag (tani; 0 = cozum yok)

    %% 1) ref noktası seçimi
    idx = nearest_ref_point(x, y, th, x_ref, y_ref, ...
                            double(idx_prev_in), 120, 0.2);
    idx_out = double(idx);

    %% tur sayacı
    threshold_high = round(0.85 * nRef);
    threshold_low  = round(0.15 * nRef);

    if double(idx_prev_in) > threshold_high && idx < threshold_low
        lap_out = double(lap_prev) + 1;

        if lap_out >= n_laps_target
            done_out  = 1;
            delta_cmd = 0;
            return;
        end
    end

    %% 2) Frenet hatasi
    xr  = x_ref(idx);
    yr  = y_ref(idx);
    thr = theta_ref(idx);

    dist2path = hypot(x - xr, y - yr);

    if dist2path > approach_dist
        thr_goto = atan2(yr - y, xr - x);
        ey = 0;
        eh = wrapToPi(th - thr_goto);
    else
        ex = x - xr;
        ey_temp = y - yr;
        ey = -sin(thr)*ex + cos(thr)*ey_temp;
        eh = wrapToPi(th - thr);
    end

    e1_dot = vy + v_const*eh;
    e2_dot = yaw_rate - v_const*kappa_ref(idx);

    x0 = [ey; e1_dot; eh; e2_dot];

    %% 3) Eğrilik
    ds_step = v_const * Ts;
    W = zeros(N_p, 1);
    for j = 1:N_p
        idx_pred_raw = idx + round(j * ds_step / ds_real);
        idx_pred = mod(idx_pred_raw - 1, nRef) + 1;
        if idx_pred < 1
            idx_pred = 1;
        end
        if idx_pred > nRef
            idx_pred = nRef;
        end
        W(j) = kappa_ref(idx_pred);
    end

    %% 4) f vektoru
    f = 2 * Gamma' * Qbar * (Phi*x0 + Psi*W);

    %% 5) Kısıt
    lb = -delta_max * ones(N_p, 1);
    ub =  delta_max * ones(N_p, 1);

    e1 = zeros(N_p,1); e1(1) = 1;
    b_upper = ddelta_max * ones(N_p,1) + u_prev*e1;
    b_lower = ddelta_max * ones(N_p,1) - u_prev*e1;
    b_ineq  = [b_upper; b_lower];

    %% 5b) Yumusak kayma acisi kisiti (Falcone 2007 (16j), on + arka aks, tek gevseme s)
    % z = [U; s]:  |alpha_f,k| <= alpha_max + s (k = 0..N_p-1),
    %              |alpha_r,k| <= alpha_max + s (k = 1..N_p),  s >= 0
    % Maliyet: + slack_w_lin*s + slack_w_quad*s^2. Dogrusal terim kisitin
    % Lagrange carpanindan buyukse (exact penalty) kisit uygulanabilirken s = 0;
    % karesel terim QP'yi s'de kesin disbukey tutar. alpha_max sonlu degilse
    % 7a QP'si aynen kullanilir. Kayma satirlarinda egrilik +-kappa_lim ile
    % kirpilir (referansin kose sivrilikleri kisiti bozmasin); maliyet/ileri
    % besleme (f) kirpilmamis W ile kalir.
    H_qp  = H_mpc;  f_qp = f;  A_qp = A_ineq_const;  b_qp = b_ineq;
    lb_qp = lb;     ub_qp = ub;
    coder.varsize('H_qp', 'f_qp', 'A_qp', 'b_qp', 'lb_qp', 'ub_qp');
    if isfinite(alpha_max)
        [M_slip, c_slip] = slip_rows(x0, W, kappa_ref(idx), Phi, Gamma, Psi, ...
                                     N_p, v_const, L - lr, lr, kappa_lim);
        n_slip = numel(c_slip);
        H_qp  = blkdiag(H_mpc, 2*slack_w_quad);         % 1/2 z'Hz -> slack_w_quad*s^2
        f_qp  = [f; slack_w_lin];
        A_qp  = [A_ineq_const, zeros(2*N_p, 1); ...
                 M_slip, -ones(n_slip, 1); ...          %  alpha - s <= alpha_max
                -M_slip, -ones(n_slip, 1)];             % -alpha - s <= alpha_max
        b_qp  = [b_ineq; alpha_max - c_slip; alpha_max + c_slip];
        lb_qp = [lb; 0];
        ub_qp = [ub; Inf];
    end

    %% 6) QP çözümü
    z_opt = zeros(numel(f_qp), 1);
    exitflag = 0;

    opts = optimoptions('quadprog', 'Display','off', ...
                        'Algorithm','interior-point-convex');
    [z_opt, ~, exitflag] = quadprog(H_qp, f_qp, ...
                                     A_qp, b_qp, ...
                                     [], [], lb_qp, ub_qp, [], opts);
    exitflag_out = double(exitflag);

    if exitflag == 1 || exitflag == 2
        delta_cmd = z_opt(1);
        if isfinite(alpha_max)
            slack_out = z_opt(end);
        end
    else
        delta_cmd = u_prev;
    end

    if delta_cmd >  delta_max, delta_cmd =  delta_max; end
    if delta_cmd < -delta_max, delta_cmd = -delta_max; end
    end

function [M, c] = slip_rows(x0, W, kappa0, Phi, Gamma, Psi, N_p, Vx, lf, lr, kappa_lim)
% On/arka lastik kayma acilarini U'nun afin fonksiyonu olarak verir:
%   alpha = M*U + c,  alpha = [alpha_f,0..N_p-1; alpha_r,1..N_p]
% Hata durumu x = [e1 e1d e2 e2d], vy = e1d - Vx*e2, r = e2d + Vx*kappa:
%   alpha_f,k = u_k - (vy_k + lf*r_k)/Vx = u_k - cf*x_k - lf*kappa_k
%   alpha_r,k =      -(vy_k - lr*r_k)/Vx =       - cr*x_k + lr*kappa_k
% x_k = Phi_k*x0 + Gamma_k*U + Psi_k*W (k >= 1; U(j) = u_{j-1}, W(j) = kappa_{j-1}
% gecisi x_{j-1} -> x_j). kappa_k secimi: k = 0 icin kappa_ref(idx) (x0'in e2d
% tanimiyla ayni), k = 1..N_p-1 icin W(k+1) (Psi'nin x_k -> x_{k+1} adiminda
% kullandigi), k = N_p icin W(N_p) (ufuk sonu).
% Egrilik burada +-kappa_lim ile kirpilir (kappa_lim = min(mu*g/Vx^2,
% tan(delta_max)/L), MPC_Setup): referansin kose sivrilikleri (pchip, kappa
% >> 1/m) hem -lf*kappa terimiyle hem Psi*W uzerinden e2d'ye girip tahmini
% alpha'yi fiziksel olmayan degerlere tasiyordu. Serbest yanit da ayni
% kirpilmis W ile kurulur (e2d ve kappa tutarli); maliyetteki W kirpilmaz.
% x0(4) = e2d0 = r - Vx*kappa0 kirpilmamis kappa ile tanimlidir; kirpilmis
% cercevede e2d0c = r - Vx*kappa0c alinir ki olculen r korunsun (aksi halde
% tum satirlar lf*(kappa0 - kappa0c) kadar kayar). |kappa0| <= kappa_lim
% iken x0c = x0.
    cf = [0, 1/Vx, -1,  lf/Vx];
    cr = [0, 1/Vx, -1, -lr/Vx];
    Wc      = max(-kappa_lim, min(kappa_lim, W));
    kappa0c = max(-kappa_lim, min(kappa_lim, kappa0));
    x0c     = x0;
    x0c(4)  = x0(4) + Vx*(kappa0 - kappa0c);  % e2d0 kirpilmis cercevede, r ayni
    Xf = Phi*x0c + Psi*Wc;                    % U'dan bagimsiz tahmin (kirpilmis W)
    kap = [Wc(2:N_p); Wc(N_p)];               % kappa_k, k = 1..N_p
    CfG = kron(eye(N_p), cf) * Gamma;   CfX = kron(eye(N_p), cf) * Xf;
    CrG = kron(eye(N_p), cr) * Gamma;   CrX = kron(eye(N_p), cr) * Xf;
    % on aks: k = 0 (x0, u_0 = U(1)); k = 1..N_p-1 (x_k, u_k = U(k+1))
    Mf = zeros(N_p, N_p);  cfv = zeros(N_p, 1);
    Mf(1, 1) = 1;          cfv(1) = -cf*x0c - lf*kappa0c;
    for k = 1:N_p-1
        Mf(k+1, :)   = -CfG(k, :);
        Mf(k+1, k+1) = Mf(k+1, k+1) + 1;
        cfv(k+1)     = -CfX(k) - lf*kap(k);
    end
    % arka aks: k = 1..N_p
    Mr  = -CrG;
    crv = -CrX + lr*kap;
    M = [Mf; Mr];
    c = [cfv; crv];
end
