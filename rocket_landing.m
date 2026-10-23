%% rocket_landing_single.m  --  run this ONE file (MATLAB R2016b+, Optimization Toolbox)
% ROBUST FUEL-OPTIMAL + PREDICTIVE CONTROL OF A ROCKET LANDING ON A MOVING SHIP (no ROS 2)
%   1. Fuel-optimal trajectory: direct transcription + trapezoidal collocation + fmincon
%   2. Four controllers compared: open-loop, open-loop+disturbances, optimal+PD, optimal+MPC
%   3. Animated landing (side-by-side, full view + zoom on the ship), saved as MP4
%   4. State/wind estimator plot
%   5. Monte-Carlo robustness study (random wind, thrust error, sensor noise)
% Switches (animation, video, Monte Carlo) are at the bottom of params().
clear; clc; close all;
p = params();

%% ---------------- 1. Optimal reference trajectory ----------------
fprintf('Solving fuel-optimal trajectory (fmincon)...\n');
tic; nom = planTrajectory(p);
fprintf('Planner exitflag = %d, planned fuel = %.1f kg  (%.1f s)\n\n', nom.exitflag, nom.fuel, toc);

%% ---------------- 2. Closed-loop comparison ----------------
dNone = makeDist(p,0,0,0);          % ideal world
dAll  = makeDist(p,1,1,1);          % wind + sensor noise + thrust error
fprintf('Simulating 4 cases...\n');
tS = tic;
res(1) = simulateLanding(p, nom, 'openloop', dNone, 'Open-loop, nominal');
res(2) = simulateLanding(p, nom, 'openloop', dAll,  'Open-loop, disturbed');
res(3) = simulateLanding(p, nom, 'pd',       dAll,  'Optimal + PD feedback');
tM = tic;
res(4) = simulateLanding(p, nom, 'mpc',      dAll,  'Optimal + MPC');
fprintf('MPC simulation took %.1f s (total sims %.1f s)\n', toc(tM), toc(tS));

fprintf('\n%-24s %7s %10s %11s %8s %9s %5s\n','Case','t [s]','posErr [m]','relVx [m/s]','vy [m/s]','fuel [kg]','OK?');
for i = 1:numel(res)
    r = res(i);
    fprintf('%-24s %7.2f %10.2f %11.2f %8.2f %9.1f %5d\n', r.name, r.tTouch, r.posErr, r.relVx, r.vy, r.fuel, r.success);
end
fprintf('(OK = landed with |pos error|<1 m, |rel. vx|<1 m/s, |vy|<2 m/s)\n');

col = {'k','r',[0.9 0.5 0],'b'};
figure('Name','Results','Position',[50 50 1300 750]);
subplot(2,3,1); hold on; grid on;
plot(nom.X(1,:), nom.X(2,:), 'g--', 'LineWidth', 1.5);
for i = 1:4, plot(res(i).X(1,:), res(i).X(2,:), 'Color', col{i}, 'LineWidth', 1.2); end
plot([-p.A p.A], [0 0], 'm', 'LineWidth', 5);
xlabel('x [m]'); ylabel('altitude [m]'); title('Trajectories (magenta = ship travel range)');
legend(['Planned', {res.name}], 'Location','best');

subplot(2,3,2); hold on; grid on;
tt = linspace(0, p.tf+p.tExtra, 500);
plot(tt, shipState(tt,p), 'm', 'LineWidth', 1.5);
for i = 1:4, plot(res(i).t, res(i).X(1,:), 'Color', col{i}); end
xlim([15 p.tf+p.tExtra]); ylim([-40 80]);
xlabel('t [s]'); ylabel('x [m]'); title('Horizontal position vs ship (zoom)');
legend(['Ship', {res.name}], 'Location','best');

subplot(2,3,3); hold on; grid on;
for i = 1:4
    Xr = referenceAt(nom, res(i).t, p);
    plot(res(i).t, res(i).X(1,:) - Xr(1,:), 'Color', col{i});
end
xlabel('t [s]'); ylabel('x - x_{ref} [m]'); title('Position tracking error');

subplot(2,3,4); hold on; grid on;
plot(nom.t, nom.U(1,:)/1e3, 'g--');
for i = 1:4, stairs(res(i).t, res(i).U(1,:)/1e3, 'Color', col{i}); end
xlabel('t [s]'); ylabel('Thrust [kN]'); title('Thrust command');

subplot(2,3,5); hold on; grid on;
plot(nom.t, rad2deg(nom.U(2,:)), 'g--');
for i = 1:4, stairs(res(i).t, rad2deg(res(i).U(2,:)), 'Color', col{i}); end
xlabel('t [s]'); ylabel('Gimbal [deg]'); title('Gimbal command');

subplot(2,3,6); hold on; grid on;
plot(nom.t, nom.X(5,:), 'g--');
for i = 1:4, plot(res(i).t, res(i).X(5,:), 'Color', col{i}); end
xlabel('t [s]'); ylabel('Mass [kg]'); title('Mass history');

%% ---------------- 3. State / wind estimator ----------------
r = res(4);
figure('Name','Estimator','Position',[100 100 1400 380]);
subplot(1,3,1); hold on; grid on;
plot(r.t, r.wind, 'k', 'LineWidth', 1.5); plot(r.t, r.dF(1,:), 'b');
xlabel('t [s]'); ylabel('Force [N]'); title('Wind force: true vs estimated'); legend('true','estimated');
subplot(1,3,2); hold on; grid on;
plot(r.t, dAll.bias*r.U(1,:), 'k', 'LineWidth', 1.5); plot(r.t, r.dF(2,:), 'b');
xlabel('t [s]'); ylabel('Force [N]'); title('Thrust error (vertical): true vs estimated'); legend('true','estimated');
subplot(1,3,3); hold on; grid on;
plot(r.t, r.X(3,:), 'k', 'LineWidth', 1.5); plot(r.t, r.xhat(3,:), 'b--');
xlabel('t [s]'); ylabel('v_x [m/s]'); title('Horizontal velocity: true vs estimated'); legend('true','estimated');

%% ---------------- 4. Animation ----------------
if p.animate
    fprintf('\nAnimating (open-loop disturbed  vs  MPC disturbed)...\n');
    animateLanding(p, res(2), res(4));
end

%% ---------------- 5. Monte-Carlo robustness ----------------
if p.runMC
    fprintf('\nMonte Carlo: %d runs x 3 controllers (MPC runs are slow, be patient)...\n', p.nMC);
    modes = {'openloop','pd','mpc'};  mnames = {'Open-loop','Optimal+PD','Optimal+MPC'};
    tMC = tic;  rng(2026);
    wScale = 0.5 + 1.5*rand(p.nMC,1);          % wind strength 0.5x..2x
    wPhase = 2*pi*rand(p.nMC,1);
    tBias  = -0.10 + 0.15*rand(p.nMC,1);       % thrust error -10 %..+5 %
    pe = zeros(p.nMC,3); rv = pe; vyv = pe; fu = pe; ok = false(p.nMC,3);
    for i = 1:p.nMC
        d = makeDist(p,1,1,1);
        d.windScale = wScale(i); d.windPhase = wPhase(i); d.bias = tBias(i); d.seed = 100+i;
        for j = 1:3
            q = simulateLanding(p, nom, modes{j}, d, mnames{j});
            pe(i,j) = q.posErr; rv(i,j) = q.relVx; vyv(i,j) = q.vy; fu(i,j) = q.fuel; ok(i,j) = q.success;
        end
        fprintf('  run %2d/%d done  (%.0f s elapsed)\n', i, p.nMC, toc(tMC));
    end
    fprintf('\n%-14s %10s %14s %14s %10s\n','Controller','success %','mean|err| [m]','max|err| [m]','fuel [kg]');
    for j = 1:3
        fprintf('%-14s %10.0f %14.2f %14.2f %10.1f\n', mnames{j}, 100*mean(ok(:,j)), mean(abs(pe(:,j))), max(abs(pe(:,j))), mean(fu(:,j)));
    end
    cc = {'r',[0.9 0.5 0],'b'};
    figure('Name','Monte Carlo','Position',[80 80 1300 420]);
    subplot(1,3,1); hold on; grid on;
    for j = 1:3, semilogy(1:p.nMC, abs(pe(:,j))+1e-3, 'o-', 'Color', cc{j}, 'LineWidth', 1.2); end
    set(gca,'YScale','log'); yline(1,'k--','1 m limit');
    xlabel('run #'); ylabel('|touchdown position error| [m]'); title('Landing accuracy per run'); legend(mnames,'Location','best');
    subplot(1,3,2);
    b = bar(100*mean(ok)); b.FaceColor = 'flat';
    for j = 1:3, b.CData(j,:) = ifelse(ischar(cc{j}), [1 0 0], cc{j}); end
    set(gca,'XTickLabel',mnames); ylabel('Success rate [%]'); ylim([0 105]); grid on; title('Successful landings');
    subplot(1,3,3);
    bar(mean(fu)); set(gca,'XTickLabel',mnames); ylabel('Mean fuel [kg]'); grid on; title('Fuel consumption');
end
fprintf('\nDone.\n');

%% =====================================================================
%  LOCAL FUNCTIONS  (must stay below the script code; MATLAB R2016b+)
%% =====================================================================

function v = ifelse(c, a, b)
if c, v = a; else, v = b; end
end

function d = makeDist(p, wind, noise, thrust)
d = struct('wind',wind, 'noise',noise, 'thrust',thrust, ...
           'windScale',1, 'windPhase',0, 'bias',p.thrustBias, 'seed',p.seed);
end

function Fw = windForce(t, p, dist)
Fw = dist.wind * dist.windScale * (p.wind.mean + p.wind.amp*sin(2*pi*t/p.wind.period + dist.windPhase));
end

function Ucmd = pdFeedback(xhat, t, nom, p)
% "Optimal + feedback" baseline: reference controls + PD correction (no prediction, no constraints).
[Xr, Ur] = referenceAt(nom, t, p);
e  = xhat - Xr;   m = max(xhat(5), p.mdry);
wn = 0.8;  kp = wn^2;  kd = 2*wn;
axc = -(kp*e(1) + kd*e(3));          % desired horizontal accel correction
ayc = -(kp*e(2) + kd*e(4));          % desired vertical accel correction
aT  = max(Ur(1)/m, 3);               % thrust accel available to the gimbal
th  = Ur(2) + axc/aT;
T   = Ur(1) + m*ayc;
Ucmd = [min(max(T,0),p.Tmax); min(max(th,-p.thMax),p.thMax)];
end

%% ---------------- animation ----------------
function animateLanding(p, resL, resR)
R = {resL, resR};
dtA = 0.1;
tA  = 0:dtA:(max(resL.t(end), resR.t(end)) + 2);
nF  = numel(tA);
P = cell(1,2);
for s = 1:2
    r = R{s};
    tq = min(tA, r.t(end));
    P{s}.X = interp1(r.t, r.X', tq, 'linear')';
    P{s}.U = interp1(r.t, r.U', tq, 'previous')';
    after = tA > r.t(end);
    if r.landed && any(after)
        [xs, vs] = shipState(tA(after), p);
        P{s}.X(1,after) = xs + r.posErr;  P{s}.X(2,after) = 0;
        P{s}.X(3,after) = vs + r.relVx;   P{s}.X(4,after) = 0;
        P{s}.U(1,after) = 0;
    end
end

fig = figure('Name','Landing animation','Position',[40 40 1250 820],'Color','w');
for s = 1:2
    axW(s) = subplot(2,2,s);   hW(s) = buildScene(axW(s));
    axZ(s) = subplot(2,2,s+2); hZ(s) = buildScene(axZ(s));
    title(axW(s), [R{s}.name ' - full view']);
    title(axZ(s), [R{s}.name ' - zoom on ship']);
end
if exist('sgtitle','file'), sgtitle('Rocket landing on a moving ship (flame shows thrust direction; rocket drawn enlarged in full view)'); end

vw = [];
if p.saveVideo
    try
        vw = VideoWriter('rocket_landing.mp4','MPEG-4');
    catch
        try, vw = VideoWriter('rocket_landing.avi','Motion JPEG AVI'); catch, vw = []; end
    end
    if ~isempty(vw), vw.FrameRate = 15; open(vw);
    else, fprintf('  (could not create a video file, continuing without saving)\n'); end
end

for j = 1:nF
    if ~ishandle(fig), break; end
    tq = tA(j);  xs = shipState(tq, p);
    for s = 1:2
        X = P{s}.X(:,j);  U = P{s}.U(:,j);
        hud = sprintf('t    = %5.1f s\nalt  = %6.1f m\nvy   = %6.1f m/s\nT    = %5.1f kN\nfuel = %5.1f kg', ...
                      tq, X(2), X(4), U(1)/1e3, p.x0(5) - X(5));
        drawScene(hW(s), axW(s), p, X, U, xs, tq, P{s}.X(1,1:j), P{s}.X(2,1:j), hud, 2.5, [-60 340], [-60 540]);
        drawScene(hZ(s), axZ(s), p, X, U, xs, tq, P{s}.X(1,1:j), P{s}.X(2,1:j), '',  1.0, xs+[-45 45], [-15 75]);
    end
    drawnow limitrate;
    if ~isempty(vw), writeVideo(vw, getframe(fig)); end
end

if ishandle(fig)
    for s = 1:2
        r = R{s};
        if r.success
            msg = sprintf('SUCCESSFUL LANDING\nerr = %.2f m, v_{rel} = %.2f m/s', r.posErr, r.relVx);  c = [0 0.5 0];
        else
            msg = sprintf('LANDING FAILED\nerr = %.1f m, vy = %.1f m/s', r.posErr, r.vy);  c = [0.8 0 0];
        end
        text(axZ(s), 0.5, 0.8, msg, 'Units','normalized', 'HorizontalAlignment','center', ...
             'FontSize', 13, 'FontWeight','bold', 'Color', c, 'BackgroundColor','w', 'EdgeColor', c);
    end
    drawnow;
    if ~isempty(vw)
        fr = getframe(fig);
        for k = 1:30, writeVideo(vw, fr); end
    end
end
if ~isempty(vw), fn = vw.Filename; close(vw); fprintf('  Saved video: %s\n', fn); end
end

function h = buildScene(ax)
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
set(ax,'Color',[0.88 0.94 1.0],'DataAspectRatio',[1 1 1]);
h.sea   = patch(ax,'XData',nan,'YData',nan,'FaceColor',[0.25 0.5 0.8],'EdgeColor','none','FaceAlpha',0.85);
h.hull  = patch(ax,'XData',nan,'YData',nan,'FaceColor',[0.25 0.27 0.3],'EdgeColor','k');
h.deck  = patch(ax,'XData',nan,'YData',nan,'FaceColor',[0.9 0.75 0.1],'EdgeColor','k');
h.wave  = plot(ax,nan,nan,'Color',[0.1 0.3 0.6],'LineWidth',1);
h.trail = plot(ax,nan,nan,'Color',[1 0.4 0],'LineWidth',1.2);
h.flame = patch(ax,'XData',nan,'YData',nan,'FaceColor',[1 0.6 0.1],'EdgeColor','none');
h.legs  = patch(ax,'XData',nan(3,2),'YData',nan(3,2),'FaceColor',[0.3 0.3 0.3],'EdgeColor','k');
h.body  = patch(ax,'XData',nan,'YData',nan,'FaceColor',[0.96 0.96 0.96],'EdgeColor','k','LineWidth',1);
h.hud   = text(ax,0.02,0.98,'','Units','normalized','VerticalAlignment','top','FontName','Courier New','FontSize',9,'BackgroundColor',[1 1 1 0.7]);
end

function drawScene(h, ax, p, X, U, xs, tq, PX, PY, hud, sc, xl, yl)
xlim(ax, xl); ylim(ax, yl);
x  = linspace(xl(1), xl(2), 200);
yw = -3 + 0.6*sin(0.15*x - 1.2*tq);
set(h.wave,'XData',x,'YData',yw);
set(h.sea, 'XData',[x xl(2) xl(1)], 'YData',[yw -80 -80]);
set(h.hull,'XData',xs+[-17 17 12 -12], 'YData',[0 0 -6 -6]);
set(h.deck,'XData',xs+[-15 15 15 -15], 'YData',[0 0 -1 -1]);
set(h.trail,'XData',PX,'YData',PY);
% rocket (origin of local frame = feet)
bx = [-1.2 1.2 1.2 0 -1.2];  by = [0.8 0.8 8 11 8];
set(h.body,'XData',X(1)+sc*bx,'YData',X(2)+sc*by);
lx = [-1.2 1.2; -3.2 3.2; -1.2 1.2];  ly = [2.5 2.5; 0 0; 1.2 1.2];
set(h.legs,'XData',X(1)+sc*lx,'YData',X(2)+sc*ly);
f = U(1)/p.Tmax;
if f > 0.01
    L = 9*f*sc;  th = U(2);
    bxp = X(1);  byp = X(2) + sc*0.8;
    tip = [bxp - L*sin(th), byp - L*cos(th)];
    set(h.flame,'XData',[bxp-sc*0.8, bxp+sc*0.8, tip(1)],'YData',[byp byp tip(2)]);
else
    set(h.flame,'XData',nan,'YData',nan);
end
set(h.hud,'String',hud);
end

% ---------- params ----------
function p = params()
% All project parameters (slide 14) in one place.
p.g = 9.80665;  p.g0 = 9.80665;  p.Isp = 300;
p.Tmax = 40e3;  p.thMax = deg2rad(20);
p.m0 = 2000;    p.mdry = 1200;
p.x0 = [300; 500; -20; -40; 2000];      % [x y vx vy m]'
% Moving ship
p.A = 15;  p.Tship = 20;  p.w = 2*pi/p.Tship;
% Planner
p.tf = 25;  p.N = 100;
p.vyTouch = -0.5;       % allowed soft-touchdown vertical speed [m/s]
p.smoothW = 5e-3;       % control smoothness weight in planner cost
% MPC
p.dtCtrl = 0.25;        % controller period [s]
p.Np     = 16;          % prediction steps (4 s horizon)
p.dtSim  = 0.01;        % plant integration step [s]
p.tExtra = 5;           % extra sim time after tf
% Disturbances / uncertainty (switched on in dist struct)
p.wind.mean = 150;  p.wind.amp = 250;  p.wind.period = 15;   % [N]
p.thrustBias  = -0.05;  % actual thrust 5 % weaker than commanded
p.thrustNoise = 0.01;   % 1 % random thrust error per control step
p.sigma = [0.5; 0.5; 0.1; 0.1; 1.0];   % sensor noise std [x y vx vy m]
% State estimator
p.L  = 0.5*ones(5,1);   % measurement correction gains
p.kd = 0.06;            % disturbance-force estimator gain (wind + thrust error)
p.sigmaY0 = 0.05;       % altimeter noise at the ground [m] (radar altimeter)
p.sigmaYk = 0.002;      % altimeter noise growth with height [m per m]
p.endGain = 14;         % MPC: extra weight on touchdown accuracy in the last seconds
p.seed = 1;
% Extras
p.animate   = true;     % show the landing animation
p.saveVideo = false;    % also write a video file (slower)
p.runMC     = false;    % Monte-Carlo study (slow: ~1 min per MPC run) -> set true when you want it
p.nMC       = 6;        % number of Monte-Carlo runs
end

% ---------- shipState ----------
function [xs, vs] = shipState(t, p)
% "Ship node": position and velocity of the moving platform.
xs = p.A*sin(p.w*t);
vs = p.A*p.w*cos(p.w*t);
end

% ---------- rocketDynamics ----------
function dX = rocketDynamics(X, U, p, Fw)
% Nonlinear plant (slide 6). X = [x y vx vy m]' (5xn), U = [T theta]' (2xn),
% Fw = disturbance force [N]: scalar (horizontal wind) or [Fx;Fy] (2x1 / 2xn).
vx = X(3,:);  vy = X(4,:);  m = X(5,:);
T  = U(1,:);  th = U(2,:);
Fx = Fw(1,:);                                   % horizontal disturbance force (wind) [N]
if size(Fw,1) > 1, Fy = Fw(2,:); else, Fy = 0; end   % vertical disturbance force [N]
dX = [ vx;
       vy;
       (T.*sin(th) + Fx)./m;
       (T.*cos(th) + Fy)./m - p.g;
      -T./(p.Isp*p.g0) ];
end

% ---------- rk4Step ----------
function Xn = rk4Step(X, U, dt, p, Fw)
k1 = rocketDynamics(X,           U, p, Fw);
k2 = rocketDynamics(X + dt/2*k1, U, p, Fw);
k3 = rocketDynamics(X + dt/2*k2, U, p, Fw);
k4 = rocketDynamics(X + dt*k3,   U, p, Fw);
Xn = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

% ---------- planTrajectory ----------
function nom = planTrajectory(p, X0, t0, tf, N)
% "Optimizer node": fuel-optimal trajectory by direct transcription with
% trapezoidal collocation, solved with fmincon. Minimizes J = -m_f.
% Decision vector: 7N variables [x y vx vy m T theta] at each node (scaled).
if nargin < 2 || isempty(X0), X0 = p.x0;  end
if nargin < 3 || isempty(t0), t0 = 0;     end
if nargin < 4 || isempty(tf), tf = p.tf;  end
if nargin < 5 || isempty(N),  N  = p.N;   end

t = linspace(t0, tf, N);  h = t(2) - t(1);
S = [300; 500; 40; 40; 2000; p.Tmax; p.thMax];       % variable scaling

% Initial guess: straight line to the touchdown state, mid thrust
[xs, vs] = shipState(tf, p);
Xf = [xs; 0; vs; p.vyTouch; X0(5) - 100];
s  = linspace(0, 1, N);
Xg = X0 + (Xf - X0).*s;
Ug = [0.5*p.Tmax*ones(1,N); zeros(1,N)];
z0 = [Xg; Ug] ./ S;

% Bounds
lb = [-inf(1,N); zeros(1,N); -inf(2,N); p.mdry*ones(1,N); zeros(1,N); -p.thMax*ones(1,N)] ./ S;
ub = [ inf(4,N); X0(5)*ones(1,N); p.Tmax*ones(1,N);  p.thMax*ones(1,N)] ./ S;

opts = optimoptions('fmincon', 'Algorithm','sqp', 'Display','final', ...
    'MaxIterations',500, 'MaxFunctionEvaluations',5e5, ...
    'OptimalityTolerance',1e-5, 'ConstraintTolerance',1e-6);

[zopt, ~, flag, out] = fmincon(@(z) planCost(z,N,p.smoothW), z0(:), ...
    [],[],[],[], lb(:), ub(:), @(z) planCon(z,p,X0,tf,h,N,S), opts);

Z = reshape(zopt, 7, N) .* S;
nom.t = t;  nom.X = Z(1:5,:);  nom.U = Z(6:7,:);
nom.exitflag = flag;  nom.output = out;
nom.fuel = X0(5) - nom.X(5,end);
end

function J = planCost(z, N, w)
Z = reshape(z, 7, N);
J = -Z(5,N) + w*sum(sum(diff(Z(6:7,:),1,2).^2));    % -m_f (+ smoothing)
end

function [c, ceq] = planCon(z, p, X0, tf, h, N, S)
Z = reshape(z, 7, N) .* S;
X = Z(1:5,:);  U = Z(6:7,:);
F = rocketDynamics(X, U, p, 0);
D = X(:,2:N) - X(:,1:N-1) - h/2*(F(:,1:N-1) + F(:,2:N));   % trapezoidal defects
D = D ./ S(1:5);
[xs, vs] = shipState(tf, p);
ceq = [ D(:);
        (X(:,1) - X0)./S(1:5);              % initial state
        (X(1,N) - xs)/S(1);                 % land on the ship
        X(2,N)/S(2);                        % altitude = 0
        (X(3,N) - vs)/S(3);                 % match ship velocity
        (X(4,N) - p.vyTouch)/S(4) ];        % soft touchdown
c = [];
end

% ---------- referenceAt ----------
function [X, U] = referenceAt(nom, t, p)
% Reference trajectory lookup. After the planned final time the reference
% keeps following the ship at touchdown condition.
tc = min(max(t, nom.t(1)), nom.t(end));
X = interp1(nom.t, nom.X', tc, 'linear')';
U = interp1(nom.t, nom.U', tc, 'linear')';
late = t > nom.t(end);
if any(late)
    [xs, vs] = shipState(t(late), p);
    X(1,late) = xs;  X(3,late) = vs;  X(2,late) = 0;  X(4,late) = p.vyTouch;
end
end

% ---------- stateEstimator ----------
function [xhat, dF] = stateEstimator(xhat, dF, Ucmd, z, p)
% "Sensor / state estimation": model-predict, then correct with the noisy
% measurement. Also estimates a slowly varying disturbance force dF = [Fx; Fy] [N]:
% Fx = wind, Fy = vertical force error (e.g. thrust shortfall), from the velocity innovations.
dt = p.dtCtrl;
xp  = rk4Step(xhat, Ucmd, dt, p, dF);
inn = z - xp;
xhat = xp + p.L .* inn;
dF = dF + p.kd * xp(5) * inn(3:4) / dt;
dF = min(max(dF, [-1500; -3000]), [1500; 3000]);
end

% ---------- mpcController ----------
function [Ucmd, uOpt] = mpcController(xhat, dF, t, nom, p, uPrev)
% "MPC node": finite-horizon nonlinear MPC (single shooting, RK4).
% Tracks the optimal reference (states + feedforward thrust/gimbal) with input
% bounds; the estimated disturbance force dF is included in the prediction model.
% Only the first action is applied (receding horizon).
Np = p.Np;  dt = p.dtCtrl;
us = [p.Tmax; p.thMax];
tk = t + (0:Np)*dt;
[Xref, Uref] = referenceAt(nom, tk, p);
Uref = Uref(:,1:Np);

if isempty(uPrev)
    u0 = Uref ./ us;   uLast = u0(:,1);
else
    u0 = [uPrev(:,2:end), uPrev(:,end)];       % warm start (shifted)
    uLast = uPrev(:,1);
end
lb = repmat([0; -1], Np, 1);
ub = repmat([1;  1], Np, 1);
opts = optimoptions('fmincon', 'Algorithm','sqp', 'Display','off', ...
    'MaxIterations',30, 'OptimalityTolerance',1e-3, 'StepTolerance',1e-6);
[uo, ~, flag] = fmincon(@(u) mpcCost(u,xhat,dF,Xref,Uref,uLast,tk,p), u0(:), ...
    [],[],[],[], lb, ub, [], opts);
if flag < 0
    uOpt = Uref ./ us;                         % fall back to the reference
else
    uOpt = reshape(uo, 2, Np);
end
Ucmd = uOpt(:,1) .* us;
end

function J = mpcCost(u, X, dF, Xref, Uref, uLast, tk, p)
Np = p.Np;  us = [p.Tmax; p.thMax];  sx = [5; 5; 1; 1];
Us = reshape(u, 2, Np);  U = Us .* us;
R = 0.5;  Rd = 0.2;  J = 0;  prev = uLast;
for k = 1:Np
    X = rk4Step(X, U(:,k), p.dtCtrl, p, dF);
    tau = max(p.tf - tk(k+1), 0);                     % time-to-go
    sc  = sx / (1 + p.endGain*exp(-tau/2));           % tighter tolerances near touchdown
    e = (X(1:4) - Xref(1:4,k+1)) ./ sc;
    wk = 1 + 9*(k == Np);                             % heavier terminal weight
    J = J + wk*(e'*e);
    du = (U(:,k) - Uref(:,k)) ./ us;                  % stay close to optimal controls
    J = J + R*(du'*du);
    dd = (Us(:,k) - prev) / 0.1;                      % move suppression (less jitter)
    J = J + Rd*(dd'*dd);
    prev = Us(:,k);
end
end

% ---------- simulateLanding ----------
function out = simulateLanding(p, nom, mode, dist, name)
% Closed-loop simulation (replaces the ROS 2 graph).
% mode: 'openloop' | 'pd' | 'mpc';  dist: struct from makeDist()
rng(dist.seed);
dtC = p.dtCtrl;  dtS = p.dtSim;  nSub = round(dtC/dtS);
tEnd = p.tf + p.tExtra;
nMax = ceil(tEnd/dtC) + 2;

X = p.x0;  t = 0;
xhat = [];  dF = [0; 0];  uPrev = [];  Ucmd = [0;0];
tl = nan(1,nMax); Xl = nan(5,nMax); Ul = nan(2,nMax); XHl = nan(5,nMax); dFl = nan(2,nMax); Wl = nan(1,nMax);
landed = false;  k = 0;

while t < tEnd - 1e-9 && ~landed
    k = k + 1;
    % --- sensor ---
    z = X;
    if dist.noise
        sg = p.sigma;  sg(2) = p.sigmaY0 + p.sigmaYk*max(X(2),0);   % altimeter: accurate near ground
        z = X + sg .* randn(5,1);
    end
    % --- controller ---
    switch mode
        case 'openloop'
            [~, Ucmd] = referenceAt(nom, t + dtC/2, p);   % mid-interval sample for ZOH
            xhat = z;
        case {'pd','mpc'}
            if k == 1, xhat = z;
            else, [xhat, dF] = stateEstimator(xhat, dF, Ucmd, z, p);
            end
            if strcmp(mode,'pd')
                Ucmd = pdFeedback(xhat, t, nom, p);
            else
                [Ucmd, uPrev] = mpcController(xhat, dF, t, nom, p, uPrev);
            end
    end
    tl(k) = t;  Xl(:,k) = X;  Ul(:,k) = Ucmd;  XHl(:,k) = xhat;
    dFl(:,k) = dF;  Wl(k) = windForce(t, p, dist);
    % --- actuator uncertainty (held over one control step) ---
    epsT = dist.thrust * (dist.bias + p.thrustNoise*randn);
    % --- plant ---
    for s = 1:nSub
        Xprev = X;  tprev = t;
        Fw = windForce(t + dtS/2, p, dist);
        T  = min(max(Ucmd(1)*(1+epsT), 0), p.Tmax);
        if X(5) <= p.mdry, T = 0; end
        th = min(max(Ucmd(2), -p.thMax), p.thMax);
        X = rk4Step(X, [T; th], dtS, p, [Fw; 0]);
        t = t + dtS;
        if X(2) <= 0                                   % touchdown
            a = Xprev(2)/(Xprev(2) - X(2));
            X = Xprev + a*(X - Xprev);
            t = tprev + a*dtS;
            landed = true;  break
        end
    end
end

k = k + 1;  tl(k) = t;  Xl(:,k) = X;  Ul(:,k) = Ucmd;  XHl(:,k) = xhat;
dFl(:,k) = dF;  Wl(k) = windForce(t, p, dist);
out.name = name;  out.t = tl(1:k);  out.X = Xl(:,1:k);
out.U = Ul(:,1:k);  out.xhat = XHl(:,1:k);  out.dF = dFl(:,1:k);  out.wind = Wl(1:k);
% --- landing monitor ---
[xs, vs] = shipState(t, p);
out.landed = landed;  out.tTouch = t;
out.posErr = X(1) - xs;
out.relVx  = X(3) - vs;
out.vy     = X(4);
out.alt    = X(2);
out.fuel   = p.x0(5) - X(5);
out.success = landed && abs(out.posErr) < 1 && abs(out.relVx) < 1 && abs(out.vy) < 2;
end
