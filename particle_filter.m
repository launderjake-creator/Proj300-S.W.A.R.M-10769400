clear; clc;

% ================= SERIAL =================
s = serialport("COM3", 115200);
configureTerminator(s, "LF");
flush(s);

% ================= CONFIG =================
N = 2000;                         % More particles = faster convergence
LM1 = [2.0, 2.0];
LM2 = [8.0, 8.0];
SENSOR_NOISE_STD = 0.3;
PROCESS_NOISE    = 0.10;
RESAMPLE_THRESH  = N / 2;
RANDOM_INJECT    = 0.02;          % 2% random particles per step
CONVERGENCE_STD  = 0.5;           % cloud spread below this = converged

% ================= INIT =================
% If you know a rough starting area, initialize tightly here.
% For global localization, spread across whole map:
particles.x = rand(1,N) * 10;
particles.y = rand(1,N) * 10;
particles.w = ones(1,N) / N;

figure('Position',[100 100 900 700]);

trail_true = []; trail_est = [];
prev_rx = []; prev_ry = [];
converged = false;

while true
    line = readline(s);
    data = sscanf(line, "%f,%f,%f,%f,%f");
    if length(data) ~= 5, continue; end
    rx=data(1); ry=data(2); rtheta=data(3); d1=data(4); d2=data(5);

    % ===== PREDICT (odometry-driven) =====
    if isempty(prev_rx), dxm=0; dym=0;
    else, dxm = rx-prev_rx; dym = ry-prev_ry;
    end
    prev_rx=rx; prev_ry=ry;
    particles.x = particles.x + dxm + randn(1,N)*PROCESS_NOISE;
    particles.y = particles.y + dym + randn(1,N)*PROCESS_NOISE;

    % ===== UPDATE (two landmarks, log-space) =====
    pred1 = sqrt((particles.x-LM1(1)).^2 + (particles.y-LM1(2)).^2);
    pred2 = sqrt((particles.x-LM2(1)).^2 + (particles.y-LM2(2)).^2);
    log_w = -0.5*((pred1-d1)/SENSOR_NOISE_STD).^2 ...
            -0.5*((pred2-d2)/SENSOR_NOISE_STD).^2;
    log_w = log_w - max(log_w);
    particles.w = exp(log_w);
    particles.w = particles.w / sum(particles.w);

    % ===== RANDOM PARTICLE INJECTION =====
    % Replace worst particles with random ones — speeds up lock-on
    n_inject = round(N * RANDOM_INJECT);
    [~, worst_idx] = mink(particles.w, n_inject);
    particles.x(worst_idx) = rand(1,n_inject)*10;
    particles.y(worst_idx) = rand(1,n_inject)*10;
    particles.w(worst_idx) = 1/N;
    particles.w = particles.w / sum(particles.w);

    % ===== RESAMPLE =====
    Neff = 1/sum(particles.w.^2);
    if Neff < RESAMPLE_THRESH
        particles = systematic_resample(particles, N);
    end

    % ===== ESTIMATE (mode-based when spread, mean when converged) =====
    spread = sqrt(var(particles.x) + var(particles.y));
    converged = spread < CONVERGENCE_STD;

    if converged
        % Converged: weighted mean is accurate
        est_x = sum(particles.x .* particles.w);
        est_y = sum(particles.y .* particles.w);
    else
        % Not converged: find densest cluster instead of averaging
        [est_x, est_y] = densest_cluster(particles);
    end

    % ===== PLOT =====
    trail_true(end+1,:)=[rx,ry];
    trail_est(end+1,:) =[est_x,est_y];

    clf; hold on;
    scatter(particles.x, particles.y, 8, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
    if size(trail_true,1)>1
        plot(trail_true(:,1), trail_true(:,2), 'g-', 'LineWidth', 1.5);
        plot(trail_est(:,1),  trail_est(:,2),  'r-', 'LineWidth', 1.5);
    end
    scatter(rx, ry,       150, 'g', 'filled', 'MarkerEdgeColor','k');
    scatter(est_x, est_y, 150, 'r', 'filled', 'MarkerEdgeColor','k');
    scatter(LM1(1), LM1(2), 200, 'k', 'p', 'filled');
    scatter(LM2(1), LM2(2), 200, 'k', 'p', 'filled');

    xlim([0 10]); ylim([0 10]); grid on; axis square;
    err = sqrt((est_x-rx)^2 + (est_y-ry)^2);
    status = "CONVERGED"; if ~converged, status = "searching..."; end
   title('test particle filter using 2 obstacles');

    drawnow limitrate;
end

% ================= DENSEST CLUSTER =================
% Fast mode estimate: grid histogram, return center of densest bin
function [mx, my] = densest_cluster(particles)
    nbins = 20;
    edges_x = linspace(0, 10, nbins+1);
    edges_y = linspace(0, 10, nbins+1);
    H = histcounts2(particles.x, particles.y, edges_x, edges_y);
    [~, idx] = max(H(:));
    [ix, iy] = ind2sub(size(H), idx);
    % Refine: weighted mean of particles in that bin
    in_bin = particles.x >= edges_x(ix)   & particles.x < edges_x(ix+1) & ...
             particles.y >= edges_y(iy)   & particles.y < edges_y(iy+1);
    if any(in_bin)
        mx = mean(particles.x(in_bin));
        my = mean(particles.y(in_bin));
    else
        mx = (edges_x(ix)+edges_x(ix+1))/2;
        my = (edges_y(iy)+edges_y(iy+1))/2;
    end
end

% ================= SYSTEMATIC RESAMPLE =================
function particles = systematic_resample(particles, N)
    positions = ((0:N-1) + rand) / N;
    cum_w = cumsum(particles.w);
    indices = zeros(1,N);
    i=1; j=1;
    while i <= N
        if positions(i) < cum_w(j), indices(i)=j; i=i+1;
        else, j=j+1; end
    end
    particles.x = particles.x(indices);
    particles.y = particles.y(indices);
    particles.w = ones(1,N) / N;
end