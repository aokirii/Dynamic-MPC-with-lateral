function [delta_cmd, ey, eh, idx_out, lap_out, done_out] = mpc_solver( ...
    x, y, th, vy, yaw_rate, u_prev, ...
    x_ref, y_ref, theta_ref, kappa_ref, ...
    idx_prev_in, lap_prev, done_prev, ...
    Phi, Gamma, Psi, H_mpc, A_ineq_const, ...
    N_p, v_const, Ts, ...
    delta_max, ddelta_max, approach_dist, ...
    Qbar, ds_real, ...
    goal_x, goal_y, goal_tol, n_laps_target)
    coder.extrinsic('quadprog','optimoptions');

    nRef = numel(x_ref);

    %% Çıkışlar
    delta_cmd = double(u_prev);
    ey        = 0;
    eh        = 0;
    idx_out   = double(idx_prev_in);
    lap_out   = double(lap_prev);
    done_out  = double(done_prev);

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

    %% 6) QP çözümü
    U_opt = zeros(N_p, 1);
    exitflag = 0;

    opts = optimoptions('quadprog', 'Display','off', ...
                        'Algorithm','interior-point-convex');
    [U_opt, ~, exitflag] = quadprog(H_mpc, f, ...
                                     A_ineq_const, b_ineq, ...
                                     [], [], lb, ub, [], opts);

    if exitflag == 1 || exitflag == 2
        delta_cmd = U_opt(1);
    else
        delta_cmd = u_prev;
    end

    if delta_cmd >  delta_max, delta_cmd =  delta_max; end
    if delta_cmd < -delta_max, delta_cmd = -delta_max; end
    end