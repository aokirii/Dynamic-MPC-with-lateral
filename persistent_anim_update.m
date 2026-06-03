function persistent_anim_update(x, y, th, delta_cmd, ey, eh)
%PERSISTENT_ANIM_UPDATE  Simulink simulasyonunda canli animasyon ve log.
%
%   * onCleanup KULLANMIYOR (Simulink ortaminda guvenilir degil).
%   * Her run yeni grafik istiyorsaniz Model > Properties > Callbacks >
%     InitFcn alanina su komutu yazin:
%         clear persistent_anim_update
%   * Log her 50 adimda bir base workspace'e 'sim_log' adiyla yazilir;
%     boylece simulasyon ortasinda durdursaniz bile veri kaybi olmaz.
%
%   Girdiler:
%     x, y, th   : arac durumu
%     delta_cmd  : MPC direksiyon komutu
%     ey, eh     : Frenet hata bilesenleri

    persistent initialized ...
               fig_h ax_h hRef hStart hGoal hTrail hSprite ...
               carImg carAlpha hasAlpha halfW halfH theta_offset ...
               trail_x trail_y idx_log ...
               t_log x_log y_log th_log delta_log ey_log eh_log ...
               step_count anim_every Ts_local log_flush_every

    %% --- Ilk cagri: figure ve sprite kur ---
    if isempty(initialized)
        x_ref     = evalin('base', 'x_ref');
        y_ref     = evalin('base', 'y_ref');
        startPt   = evalin('base', 'sim_params.startPt');
        goal      = evalin('base', 'sim_params.goal');
        T_end     = evalin('base', 'T_end');
        Ts_local  = evalin('base', 'Ts');

        step_count       = 0;
        anim_every       = 4;     % her 4 adimda bir gorsel guncelle
        log_flush_every  = 50;    % her 50 adimda bir workspace'e yaz
        theta_offset     = pi/2;

        fig_h = figure('Color','w','Name','MPC Trajectory Tracking', ...
                       'Position',[100 100 900 700]);
        ax_h  = axes('Parent', fig_h);
        hold(ax_h,'on'); grid(ax_h,'on'); axis(ax_h,'equal');

        hRef   = plot(ax_h, x_ref, y_ref, 'b--', 'LineWidth',1.5);
        hStart = plot(ax_h, startPt(1), startPt(2), 'go','MarkerSize',10, ...
                      'LineWidth',2, 'MarkerFaceColor','g');
        hGoal  = plot(ax_h, goal(1), goal(2), 'rx','MarkerSize',12,'LineWidth',2);
        hTrail = plot(ax_h, nan, nan, 'm-','LineWidth',1.8);

        title(ax_h, 'MPC Live Tracking');
        xlabel(ax_h, 'x [m]'); ylabel(ax_h, 'y [m]');
        legend(ax_h, [hRef hStart hGoal hTrail], ...
               {'Referans','Start','Goal','Arac'}, 'Location','best');

        pad = 30;
        xlim(ax_h, [min(x_ref)-pad, max(x_ref)+pad]);
        ylim(ax_h, [min(y_ref)-pad, max(y_ref)+pad]);

        % Sprite (varsa)
        iconPath = '/Users/kasimesen/Desktop/Tasarım/car.png';
        hasAlpha = false;
        try
            [carImg, ~, carAlpha] = imread(iconPath);
            carImg = im2double(carImg);
            hasAlpha = ~isempty(carAlpha);
            if hasAlpha
                carAlpha = im2double(carAlpha);
            end

            mapScale = min(max(x_ref)-min(x_ref), max(y_ref)-min(y_ref));
            carSize  = 0.04 * mapScale;
            halfW    = carSize/2;
            halfH    = carSize/2;

            carRot = imrotate(carImg, -rad2deg(th + theta_offset), ...
                              'bilinear','crop');
            hSprite = image(ax_h, [x-halfW, x+halfW], ...
                                  [y-halfH, y+halfH], carRot);
            if hasAlpha
                alphaRot = imrotate(carAlpha, -rad2deg(th + theta_offset), ...
                                    'bilinear','crop');
                set(hSprite, 'AlphaData', alphaRot);
            end
            uistack(hSprite,'top');
        catch
            hSprite = plot(ax_h, x, y, 'ko', 'MarkerSize',12, ...
                           'MarkerFaceColor','y','LineWidth',2);
            halfW = 0; halfH = 0;
        end

        % Log dizileri
        N_max = round(T_end / Ts_local) + 10;
        t_log     = nan(N_max, 1);
        x_log     = nan(N_max, 1);
        y_log     = nan(N_max, 1);
        th_log    = nan(N_max, 1);
        delta_log = nan(N_max, 1);
        ey_log    = nan(N_max, 1);
        eh_log    = nan(N_max, 1);
        trail_x   = nan(N_max, 1);
        trail_y   = nan(N_max, 1);
        idx_log   = 0;

        initialized = true;
    end

    %% --- Sayac + log ---
    step_count = step_count + 1;
    t_now      = (step_count - 1) * Ts_local;

    idx_log = idx_log + 1;
    if idx_log <= numel(t_log)
        t_log(idx_log)     = t_now;
        x_log(idx_log)     = x;
        y_log(idx_log)     = y;
        th_log(idx_log)    = th;
        delta_log(idx_log) = delta_cmd;
        ey_log(idx_log)    = ey;
        eh_log(idx_log)    = eh;
        trail_x(idx_log)   = x;
        trail_y(idx_log)   = y;
    end

    %% --- Animasyonu guncelle ---
    if mod(step_count, anim_every) == 0
        if ishandle(hTrail)
            set(hTrail,'XData', trail_x(1:idx_log), ...
                       'YData', trail_y(1:idx_log));
        end
        if ishandle(hSprite)
            try
                if hasAlpha
                    set(hSprite, 'XData', [x-halfW x+halfW], ...
                                 'YData', [y-halfH y+halfH], ...
                                 'CData', imrotate(carImg, ...
                                          -rad2deg(th + theta_offset), ...
                                          'bilinear','crop'));
                    set(hSprite, 'AlphaData', imrotate(carAlpha, ...
                                          -rad2deg(th + theta_offset), ...
                                          'bilinear','crop'));
                else
                    set(hSprite, 'XData', x, 'YData', y);
                end
            catch
            end
        end
        drawnow limitrate;
    end

    %% --- Log'u workspace'e periyodik yaz ---
    % Bu sayede sim hata verse veya kullanici durdursa bile veri kaybolmaz
    if mod(step_count, log_flush_every) == 0
        K = idx_log;
        sim_log.t     = t_log(1:K);
        sim_log.x     = x_log(1:K);
        sim_log.y     = y_log(1:K);
        sim_log.theta = th_log(1:K);
        sim_log.delta = delta_log(1:K);
        sim_log.ey    = ey_log(1:K);
        sim_log.eh    = eh_log(1:K);
        assignin('base', 'sim_log', sim_log);
    end
end