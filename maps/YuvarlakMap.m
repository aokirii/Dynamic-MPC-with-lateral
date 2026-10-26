% Çember parametreleri
R = 120;
C = [250 250];

t = linspace(0, 2*pi, 801);
t(end) = [];   

x = C(1) + R*cos(t);
y = C(2) + R*sin(t);

% başlangıç ve bitiş dereceleri
theta_start_deg = 40;
theta_end_deg   = 210;

ts = deg2rad(theta_start_deg);
te = deg2rad(theta_end_deg);

startPt = [C(1) + R*cos(ts), C(2) + R*sin(ts)];
goal    = [C(1) + R*cos(te), C(2) + R*sin(te)];

% Çizim
figure('Color','w'); hold on; grid on; axis equal;
plot(x, y, '--', 'LineWidth', 2);
plot(startPt(1), startPt(2), 'o', 'MarkerSize', 9, 'LineWidth', 2);
plot(goal(1),    goal(2),    'x', 'MarkerSize',11, 'LineWidth', 2);

pad = 30;
xlim([C(1)-R-pad, C(1)+R+pad]);
ylim([C(2)-R-pad, C(2)+R+pad]);

title('Tam Çember Map + Ayarlanabilir Start/End');
xlabel('x'); ylabel('y');
legend('Yol (çember)','Start','End');

% Waypoints
waypoints = [x(:), y(:)];
waypoints = waypoints(all(isfinite(waypoints),2),:);

% Ardışık aynı noktaları temizle (ekstra garanti)
d = sqrt(sum(diff(waypoints).^2,2));
waypoints = waypoints([true; d > 1e-9],:);
assignin('base','startPt',startPt);
assignin('base','goal',goal);