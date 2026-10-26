%% Rectangle parameters
C = [100 100];
W = 80;  H = 80;
x0 = C(1) - W/2;  x1 = C(1) + W/2;
y0 = C(2) - H/2;  y1 = C(2) + H/2;
P = 2*(W+H);  % perimeter length

%% Start noktası (kenar + oran)
% edges: 1=bottom, 2=right, 3=top, 4=left
start_edge = 2;  start_s = 0.25;

point_on_edge = @(e,s) ...
    (e==1)*[x0+s*W, y0] + ...
    (e==2)*[x1, y0+s*H] + ...
    (e==3)*[x1-s*W, y1] + ...
    (e==4)*[x0, y1-s*H];

S = point_on_edge(start_edge, start_s);

%% Convert (edge,s) -> perimeter coordinate p in [0,P)
edge_to_p = @(e,s) ...
    (e==1)*(0      + s*W) + ...
    (e==2)*(W      + s*H) + ...
    (e==3)*(W+H    + s*W) + ...
    (e==4)*(2*W+H  + s*H);

pS = edge_to_p(start_edge, start_s);

%% TAM TUR: başlangıçtan başlayıp tam çevre kadar git (kapalı döngü)
N = 1500;
p = pS + linspace(0, P, N).';   % tam bir tur (P = çevre)
p = mod(p, P);

%% Map p -> (x,y)
x = zeros(N,1); y = zeros(N,1);
m1 = p < W;
m2 = p >= W & p < W+H;
m3 = p >= W+H & p < 2*W+H;
m4 = p >= 2*W+H;

x(m1) = x0 + p(m1);                 y(m1) = y0;
x(m2) = x1;                         y(m2) = y0 + (p(m2)-W);
x(m3) = x1 - (p(m3)-(W+H));         y(m3) = y1;
x(m4) = x0;                         y(m4) = y1 - (p(m4)-(2*W+H));

waypoints = [x y];

%% Export
startPt = S;
goal    = S;  % tam tur: hedef = başlangıç
assignin('base','waypoints',waypoints);
assignin('base','startPt',startPt);
assignin('base','goal',goal);

%% Quick plot
figure('Color','w'); hold on; grid on; axis equal;
plot([x0 x1 x1 x0 x0],[y0 y0 y1 y1 y0],'k--','LineWidth',1);
plot(waypoints(:,1),waypoints(:,2),'b-','LineWidth',2);
plot(S(1),S(2),'ro','MarkerSize',10,'LineWidth',2);
title('Rectangle Map (Full Loop)');
xlabel('x'); ylabel('y');
legend('Sınır','Yol','Start = Goal','Location','best');
