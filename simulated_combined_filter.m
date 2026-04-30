clear; clc;

% ================= SERIAL =================
s = serialport("COM3", 115200);
configureTerminator(s, "LF");
flush(s);

% ================= CONFIG =================
NUM_ROBOTS = 4;
N = 1500;
LM1 = [2.0, 2.0];
LM2 = [8.0, 8.0];
SENSOR_NOISE_STD = 0.3;
PROCESS_NOISE    = 0.10;
RESAMPLE_THRESH  = N / 2;
RANDOM_INJECT    = 0.02;
CONVERGENCE_STD  = 0.5;

% Colours for the 4 robots
COLOURS = [
    0.20 0.40 1.00;    % blue
    1.00 0.40 0.20;    % orange
    0.20 0.80 0.30;    % green
    0.80 0.20 0.80;    % purple
];

% ================= INIT 4 FILTERS =================
filters = struct([]);
for r = 1:NUM_ROBOTS
    filters(r).x = rand(1,N) * 10;
    filters(r).y = rand(1,N) * 10;
    filters(r).w = ones(1,N) / N;
    filters(r).prev_rx = [];
    filters(r).prev_ry = [];
    filters(r).trail_true = [];
    filters(r).trail_est  = [];
end

% ================= FIGURE =================
figure('Position',[100 100 950 850], ...
       'Name','test particle filter using 2 obstacles — 4 robots', ...
       'NumberTitle','off');

% ================= MAIN LOOP =================
while true
    line = readline(s);
    data = sscanf(line, [repmat('%f,',1,19) '%f']);
    if length(data) ~= 20, continue; end

    % Unpack: reshape to 5 fields × 4 robots
    D = reshape(data, 5, NUM_ROBOTS);   % rows: x,y,theta,d1,d2

    clf; hold on;

    for r = 1:NUM_ROBOTS
        rx=D(1,r); ry=D(2,r); rtheta=D(3,r); d1=D(4,r); d2=D(5,r);
        f = filters(r);

        % ===== PREDICT (odometry-driven) =====
        if isempty(f.prev_rx), dxm=0; dym=0;
        else, dxm=rx-f.prev_rx; dym=ry-f.prev_ry;
        end
        f.prev_rx=rx; f.prev_ry=ry;
        f.x = f.x + dxm + randn(1,N)*PROCESS_NOISE;
        f.y = f.y + dym + randn(1,N)*PROCESS_NOISE;

        % ===== UPDATE =====
        pred1 = sqrt((f.x-LM1(1)).^2 + (f.y-LM1(2)).^2);
        pred2 = sqrt((f.x-LM2(1)).^2 + (f.y-LM2(2)).^2);
        log_w = -0.5*((pred1-d1)/SENSOR_NOISE_STD).^2 ...
                -0.5*((pred2-d2)/SENSOR_NOISE_STD).^2;
        log_w = log_w - max(log_w);
        f.w = exp(log_w);
        f.w = f.w / sum(f.w);

        % ===== RANDOM INJECTION =====
        n_inject = round(N * RANDOM_INJECT);
        [~, worst_idx] = mink(f.w, n_inject);
        f.x(worst_idx) = rand(1,n_inject)*10;
        f.y(worst_idx) = rand(1,n_inject)*10;
        f.w(worst_idx) = 1/N;
        f.w = f.w / sum(f.w);

        % ===== RESAMPLE =====
        Neff = 1/sum(f.w.^2);
        if Neff < RESAMPLE_THRESH
            f = systematic_resample_struct(f, N);
        end

        % ===== ESTIMATE =====
        spread = sqrt(var(f.x) + var(f.y));
        converged = spread < CONVERGENCE_STD;
        if converged
            est_x = sum(f.x .* f.w);
            est_y = sum(f.y .* f.w);
        else
            [est_x, est_y] = densest_cluster(f);
        end

        f.trail_true(end+1,:) = [rx, ry];
        f.trail_est(end+1,:)  = [est_x, est_y];

        % ===== PLOT =====
        col = COLOURS(r,:);
        % particles (low alpha so they don't dominate)
        scatter(f.x, f.y, 6, col, 'filled', 'MarkerFaceAlpha', 0.15);
        % trails
        if size(f.trail_true,1) > 1
            plot(f.trail_true(:,1), f.trail_true(:,2), '-', ...
                 'Color', col*0.6, 'LineWidth', 1.2);
            plot(f.trail_est(:,1),  f.trail_est(:,2),  '--', ...
                 'Color', col,      'LineWidth', 1.5);
        end
        % current true + estimate
        scatter(rx, ry, 120, col*0.6, 'filled', 'MarkerEdgeColor','k');
        scatter(est_x, est_y, 120, col, '^', 'filled', 'MarkerEdgeColor','k');

        % save filter back
        filters(r) = f;
    end

    % Landmarks
    scatter(LM1(1), LM1(2), 250, 'k', 'p', 'filled');
    scatter(LM2(1), LM2(2), 250, 'k', 'p', 'filled');

    xlim([0 10]); ylim([0 10]); grid on; axis square;
    title('test particle filter using 2 obstacles');
    xlabel('x (m)'); ylabel('y (m)');

    % Simple custom legend
    legend_entries = gobjects(NUM_ROBOTS+1, 1);
    for r = 1:NUM_ROBOTS
        legend_entries(r) = plot(NaN, NaN, 's', ...
            'MarkerFaceColor', COLOURS(r,:), 'MarkerEdgeColor','k', ...
            'MarkerSize', 10);
    end
    legend_entries(NUM_ROBOTS+1) = plot(NaN, NaN, 'kp', ...
        'MarkerFaceColor','k', 'MarkerSize', 12);
    legend(legend_entries, ...
        {'Robot 1 (circle)','Robot 2 (square)','Robot 3 (fig-8)', ...
         'Robot 4 (lawnmower)','Landmarks'}, ...
        'Location','eastoutside');

    drawnow limitrate;
end

% ================= HELPERS =================
function [mx, my] = densest_cluster(f)
    nbins = 20;
    edges_x = linspace(0, 10, nbins+1);
    edges_y = linspace(0, 10, nbins+1);
    H = histcounts2(f.x, f.y, edges_x, edges_y);
    [~, idx] = max(H(:));
    [ix, iy] = ind2sub(size(H), idx);
    in_bin = f.x >= edges_x(ix) & f.x < edges_x(ix+1) & ...
             f.y >= edges_y(iy) & f.y < edges_y(iy+1);
    if any(in_bin)
        mx = mean(f.x(in_bin));
        my = mean(f.y(in_bin));
    else
        mx = (edges_x(ix)+edges_x(ix+1))/2;
        my = (edges_y(iy)+edges_y(iy+1))/2;
    end
end

function f = systematic_resample_struct(f, N)
    positions = ((0:N-1) + rand) / N;
    cum_w = cumsum(f.w);
    indices = zeros(1,N);
    i=1; j=1;
    while i <= N
        if positions(i) < cum_w(j), indices(i)=j; i=i+1;
        else, j=j+1; end
    end
    f.x = f.x(indices);
    f.y = f.y(indices);
    f.w = ones(1,N) / N;
end