% Harita sınırları
xlim_world = [0 100];
ylim_world = [0 100];
figure('Color','w'); hold on; grid on; axis equal;
xlim(xlim_world); ylim(ylim_world);
title({'Sol tık: nokta ekle', ...
'Backspace: son noktayı sil | Sağ tık veya Enter: bitir'});
xlabel('x'); ylabel('y');
x = []; y = [];
% İlk boş çizgi objesini oluştur
hPath  = plot(nan, nan, '--', 'LineWidth', 2);
hStart = plot(nan, nan, 'o',  'MarkerSize', 9, 'LineWidth', 2);
hEnd   = plot(nan, nan, 'x',  'MarkerSize',11, 'LineWidth', 2);
while true
    [xi, yi, button] = ginput(1);
    % Enter: ginput boş dönebilir
    if isempty(button)
        break;
    end
    if button == 1
        % Sol tık: nokta ekle
        x(end+1,1) = xi;
        y(end+1,1) = yi;
    elseif button == 8
        % Backspace: son noktayı sil
        if ~isempty(x)
            x(end) = []; y(end) = [];
        end
    else
        % Sağ tık: bitir
        break;
    end
    % Çizim güncelle
    set(hPath, 'XData', x, 'YData', y);
    if ~isempty(x)
        set(hStart, 'XData', x(1),  'YData', y(1));
        set(hEnd,   'XData', x(end),'YData', y(end));
    else
        set(hStart, 'XData', nan, 'YData', nan);
        set(hEnd,   'XData', nan, 'YData', nan);
    end
    drawnow;
end

% En az 2 nokta kontrolü
if numel(x) < 2
    warning('Yeterli nokta seçilmedi (en az 2 nokta önerilir).');
end

waypoints = [x(:), y(:)];
waypoints = waypoints(all(isfinite(waypoints),2),:);
waypoints = unique(waypoints,'rows','stable');
if size(waypoints,1) < 2, error('Waypoints çok az.'); end

%% OTOMATİK YOLU KAPAT
% Son nokta ile ilk nokta arasındaki mesafe kontrol et
close_tol = 3.0;  % bu mesafeden yakınsa zaten kapalı say
d_close = hypot(waypoints(end,1)-waypoints(1,1), waypoints(end,2)-waypoints(1,2));

if d_close > close_tol
    % Son noktadan ilk noktaya ara noktalar ekle (düz çizgi ile bağla)
    n_bridge = max(5, round(d_close / 2));  % her 2 birimde bir nokta
    x_bridge = linspace(waypoints(end,1), waypoints(1,1), n_bridge+1)';
    y_bridge = linspace(waypoints(end,2), waypoints(1,2), n_bridge+1)';
    % İlk ve son noktayı tekrar eklememek için 2:end-1
    waypoints = [waypoints; x_bridge(2:end-1), y_bridge(2:end-1)];
    fprintf('Yol otomatik kapatıldı (%.1f m köprü, %d ara nokta)\n', d_close, n_bridge-2);
end

% Son noktayı ilk noktaya eşitle (tam kapanma)
waypoints(end+1,:) = waypoints(1,:);

%% Çizimi güncelle
set(hPath, 'XData', waypoints(:,1), 'YData', waypoints(:,2));
title('Hand Figure Map (Kapalı Döngü)');
drawnow;

%% Export
startPt = waypoints(1,:);
goal    = waypoints(1,:);  % kapalı döngü: hedef = başlangıç
assignin('base','waypoints',waypoints);
assignin('base','startPt',startPt);
assignin('base','goal',goal);