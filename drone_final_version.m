clear; clc; close all;

% seed 13 to avoid randomness and make the experiment repeatible
rng(13);

%% ===== Park & grid =====
Park_km = 50;           % Number of n by n Km of the park.
Cell_km = 1;            % How many Km are in a single cell.
N = Park_km / Cell_km;  % Number of the square matrix of cells.

Visited = zeros(N,N);   % Matrix that shows where the drone has been (cell visits).

%% ===== Mission selection (fire vs missing) =====
% 1 = fire mission (5x5), 2 = missing person mission (3x3)
mission = input('Mission? Type 1 for FIRE (5x5) or 2 for MISSING PERSON (3x3): ');

if mission == 1
    Detect_size = 5;        % fires -> 5x5
    Lane_spacing_km = 4;    % rule for 5x5
    missionName = "FIRE";
else
    Detect_size = 3;        % missing -> 3x3
    Lane_spacing_km = 2;    % rule for 3x3
    missionName = "MISSING";
end

%% ===== Detection =====
K = ones(Detect_size);  % Square matrix of windows (Kernel).

%% ===== Random task maps (terrain events) =====
% Ask user how many fires and missing persons
numFires  = input('How many FIRES do you want in the park? (integer): ');
numPeople = input('How many MISSING PERSONS do you want in the park? (integer): ');

% Clamp to maximum cells
numFires  = max(0, min(numFires,  N*N));
numPeople = max(0, min(numPeople, N*N));

FireMap   = false(N,N);
PersonMap = false(N,N);

% Randomly place fires
if numFires > 0
    idxF = randperm(N*N, numFires);
    FireMap(idxF) = true;
end

% Randomly place missing persons
if numPeople > 0
    idxP = randperm(N*N, numPeople);
    PersonMap(idxP) = true;
end

% Detection result maps (accumulative through time)
FiresDetected   = false(N,N);
PersonsDetected = false(N,N);

%% ===== Zigzag waypoints =====
Margin_km = 1;          % How near the drone will be from the limit of the park.
wp = makeZigZagWaypoints(Park_km, Lane_spacing_km, Margin_km); % waypoints



%% ===== Time =====
dt = 0.5;
T  = 4900;

if mission == 2
    T = 8900;
end

t  = 0:dt:T;


%% ===== GPS/Tracking model =====
Gps_update_s = 15;       % GPS updates every 15 s (given in problem)

P_gps_ok   = 0.7;
P_gps_bad  = 0.2;
P_gps_lost = 0.1;

Track_err_max = 0.25;   % km
Gps_ok_err_max = 0.01;  % km
Gps_bad_err_max = 0.50; % km

GPS_OK   = 1;
GPS_BAD  = 2;
GPS_LOST = 3;

GPS_state = GPS_OK;     % Initial GPS state
z_gps = [NaN; NaN];     % last GPS measurement (NaN means unavailable)

%% ===== Wind disturbance model =====
Ax = 0.0002;   fx = 0.015;
Ay = 0.0002;   fy = 0.013;

noiseA = 0.001;

wind_ax = @(tt) Ax*sin(2*pi*fx*tt) + noiseA*randn; % (seed)
wind_ay = @(tt) Ay*sin(2*pi*fy*tt) + noiseA*randn;

%% ===== PID (position -> acceleration) =====
Kp = 0.09;   % Proportional term
Ki = 0.001;  % Integral term
Kd = 0.01;   % Derivative term

integral_error = [0;0];
previous_error = [0;0];

der_filt = [0;0];
tau_d = 2.0; % seconds (1–5s typical)
alpha_d = dt/(tau_d + dt);

u_max = 0.08;   % accel saturation [km/s^2]
v_max = 0.15;   % speed saturation [km/s]

Imax = 20;      % [km*s]

%% ===== Waypoint switching robustness =====
reach_tol = 0.75;            % km
previous_target = wp(1,:)';  % for "passed" logic

%% ===== Plant state (truth) =====
x  = wp(1,1);   % Initial drone X-coordinate
y  = wp(1,2);   % Initial drone Y-coordinate
vx = 0;         % Initial velocity in X
vy = 0;         % Initial velocity in Y

wp_idx = 2;     % Next waypoint to chase
final_wp = wp(end,:)';
finished = false;

%% ===== Logs =====
traj = nan(numel(t),2);

fig = figure(1);
set(fig,'Color','w');

for k = 1:numel(t)
    tt = t(k);

    %% ----- Current target waypoint -----
    if wp_idx <= size(wp,1)
        target = wp(wp_idx,:)';
    else
        target = final_wp;   % en vez de [x;y]
    end

    if wp_idx > size(wp,1)
        finished = true;
    end

    z_track = [x; y] + [randUniformSym(Track_err_max); randUniformSym(Track_err_max)];

    if mod(tt, Gps_update_s) == 0
        GPS_state = sampleGPSState(P_gps_ok, P_gps_bad, P_gps_lost);

        if GPS_state == GPS_OK
            z_gps = [x; y] + [randUniformSym(Gps_ok_err_max); randUniformSym(Gps_ok_err_max)];
        elseif GPS_state == GPS_BAD
            z_gps = [x; y] + [randUniformSym(Gps_bad_err_max); randUniformSym(Gps_bad_err_max)];
        else
            z_gps = [NaN; NaN];
        end
    end

    %% ----- Measurement selection -----
    if GPS_state ~= GPS_LOST && ~any(isnan(z_gps))
        estimated_position = z_gps;
    else
        estimated_position = z_track;
    end

    if finished
    % stop: no control, no wind, and set velocities to zero
        u = [0;0];
        vx = 0; vy = 0;
        ax = 0; ay = 0;
    else
        % ----- PID control (uses estimated position) -----
        error = target - estimated_position;
    
        integral_error = integral_error + error * dt;
        integral_error = max(min(integral_error, Imax), -Imax);
    
        der_raw = (error - previous_error)/dt;
        der_filt = (1 - alpha_d)*der_filt + alpha_d*der_raw;
        derivative = der_filt;
        previous_error = error;
    
        u = Kp * error + Ki * integral_error + Kd * derivative;
        u = max(min(u, u_max), -u_max);
    
        % ----- Plant update with wind disturbance -----
        ax = u(1) + wind_ax(tt);
        ay = u(2) + wind_ay(tt);
    
        vx = vx + ax*dt;
        vy = vy + ay*dt;
    
        vnorm = norm([vx;vy]);
        if vnorm > v_max
            vv = [vx;vy] * (v_max / vnorm);
            vx = vv(1); vy = vv(2);
        end
    end

    % Update position (always)
    x = x + vx*dt;
    y = y + vy*dt;

    x = min(max(x,0), Park_km);
    y = min(max(y,0), Park_km);

    traj(k,:) = [x y];

    %% ----- Waypoint switching: distance OR passed -----
    a = previous_target;
    b = target;
    p = estimated_position;

    dist = norm(p - b);

    direction_vector = b - a;
    if norm(direction_vector) > 1e-6
        passed = dot(p - b, direction_vector) > 0;
    else
        passed = false;
    end

    if wp_idx <= size(wp,1) && (dist < reach_tol || passed)
        previous_target = target;
        wp_idx = wp_idx + 1;

        integral_error = [0;0];
        previous_error = [0;0];
    end

    %% ----- Coverage -----
    [i,j] = Position2Cell(x, y, Cell_km, N);
    Visited(i,j) = Visited(i,j) + 1;

    coverage = conv2(Visited, K, 'same');
    covered  = coverage > 0;

    %% ----- Detection of events (fires / missing persons) -----
    FiresDetected   = FiresDetected   | (FireMap   & covered);
    PersonsDetected = PersonsDetected | (PersonMap & covered);

    %% ----- Plot (4 plots) -----
    if mod(k,10)==0 || k==1
        clf(fig);

        % 1) Trajectory (continuous)
        subplot(2,2,1);
        plot(wp(:,1), wp(:,2), 'k.-'); hold on;
        plot(traj(1:k,1), traj(1:k,2), 'b', 'LineWidth', 1.5);
        plot(x, y, 'ro', 'LineWidth', 1.5);
        plot(target(1), target(2), 'go', 'LineWidth', 2);
        grid on; axis equal;
        xlim([0 Park_km]); ylim([0 Park_km]);
        title("Trajectory tracking (" + missionName + ")");
        xlabel("x [km]"); ylabel("y [km]");

        if GPS_state ~= GPS_LOST && ~any(isnan(z_gps))
            txt = "Sensor: GPS";
        else
            txt = "Sensor: Tracking";
        end
        text(1, Park_km-1, txt, 'FontWeight','bold');

        % 2) Visited cells + events overlay (always show both)
        subplot(2,2,2);
        imagesc(Visited > 0);
        axis image; set(gca,'YDir','normal');
        title("Visited cells + events");
        xlabel("X cell"); ylabel("Y cell");
        hold on;

        plot(j, i, 'wo', 'MarkerSize', 7, 'LineWidth', 1.5);

        [rf, cf] = find(FireMap);
        plot(cf, rf, 'r.', 'MarkerSize', 10);

        [rp, cp] = find(PersonMap);
        plot(cp, rp, 'c.', 'MarkerSize', 10);

        hold off;

        % 3) Binary coverage + ONLY mission detections as '+'
        subplot(2,2,3);
        imagesc(covered);
        axis image; set(gca,'YDir','normal');
        title(sprintf("Coverage (binary) detect %dx%d", Detect_size, Detect_size));
        xlabel("X cell"); ylabel("Y cell");
        hold on;

        if mission == 1
            % FIRE mission -> show ONLY fires detected as red '+'
            [rdf, cdf] = find(FiresDetected);
            plot(cdf, rdf, 'r+', 'MarkerSize', 7, 'LineWidth', 1.5);
        else
            % MISSING mission -> show ONLY persons detected as cyan '+'
            [rdp, cdp] = find(PersonsDetected);
            plot(cdp, rdp, 'c+', 'MarkerSize', 7, 'LineWidth', 1.5);
        end

        hold off;

        % 4) Raw coverage counts (conv2 result)
        subplot(2,2,4);
        imagesc(coverage); axis image; set(gca,'YDir','normal');
        colorbar;
        title('Coverage counts (conv2 result)');
        xlabel('X cell'); ylabel('Y cell');

        % Status text: show detected/total for both (still useful)
        detectedF = nnz(FiresDetected);
        detectedP = nnz(PersonsDetected);
        totalF = nnz(FireMap);
        totalP = nnz(PersonMap);
        sgtitle(sprintf("Detected Fires: %d/%d | Detected People: %d/%d", detectedF, totalF, detectedP, totalP));

        drawnow limitrate;
    end
end

disp("Done.");

fprintf("Fires placed: %d | Fires detected: %d\n", nnz(FireMap), nnz(FiresDetected));
fprintf("People placed: %d | People detected: %d\n", nnz(PersonMap), nnz(PersonsDetected));
