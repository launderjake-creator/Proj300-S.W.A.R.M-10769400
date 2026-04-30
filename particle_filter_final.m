clear; clc;

% ================= SERIAL =================
s = serialport("COM12", 115200);
configureTerminator(s, "LF");
s.Timeout = 10;
flush(s);

fprintf('\n╔════════════════════════════════════════════╗\n');
fprintf('║   PARTICLE FILTER - MANUAL START MODE     ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');
fprintf('INSTRUCTIONS:\n');
fprintf('1. Make sure both ESPs are powered and connected\n');
fprintf('2. Check Central ESP serial monitor is open\n');
fprintf('3. Verify Receiver ESP completed calibration\n');
fprintf('4. Press ENTER when ready to start Mission 1...\n\n');

input('Press ENTER to continue: ', 's');

% Send Mission 1 command
writeline(s, "1");
fprintf('\n✅ Mission 1 (Sensors) command sent\n');
fprintf('⏳ Waiting for sensor data (up to 10 seconds)...\n\n');

% Verify data is flowing - MORE ATTEMPTS, LONGER TIMEOUT
fprintf('Listening for sensor data:\n');
dataOK = false;
for attempt = 1:20  % 20 attempts instead of 10
    if s.NumBytesAvailable > 0
        testLine = readline(s);
        
        % Show all received lines for debugging
        if startsWith(testLine, "#")
            fprintf('  [comment] %s\n', testLine);
        else
            fprintf('  [DATA] %s\n', testLine);
            dataOK = true;
            fprintf('\n✅ Sensor data confirmed!\n\n');
            break;
        end
    else
        fprintf('.');  % Progress indicator
    end
    pause(0.5);  % Wait 0.5s between attempts = 10 seconds total
end
fprintf('\n');

if ~dataOK
    fprintf('\n❌ ERROR: No sensor data received after 10 seconds!\n\n');
    fprintf('Troubleshooting:\n');
    fprintf('  1. Open Receiver ESP serial monitor:\n');
    fprintf('     - Did it receive " COMMAND RECEIVED: MISSION_SENSORS"?\n');
    fprintf('     - Do you see " Sent | H:... | US:..." messages?\n\n');
    fprintf('  2. Open Central ESP serial monitor:\n');
    fprintf('     - Do you see CSV data lines (numbers with commas)?\n');
    fprintf('     - Or only # comment lines?\n\n');
    fprintf('  3. Try MANUALLY:\n');
    fprintf('     - In Central ESP serial monitor, type 1 and press Enter\n');
    fprintf('     - Check if CSV data starts flowing\n\n');
    fprintf('  4. If still no data:\n');
    fprintf('     - Check ESP-NOW MAC addresses are correct\n');
    fprintf('     - Verify both ESPs are on same WiFi channel\n');
    fprintf('     - Try resetting both ESPs and running again\n\n');
    error('Setup failed - no sensor data detected');
end

% CLEAR BUFFER - throw away any old data
flush(s);
pause(0.5);

fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║       PARTICLE FILTER STARTING NOW         ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');
fprintf('Keyboard Controls (click figure window first):\n');
fprintf('  D = Dock (Mission 2)\n');
fprintf('  U = Undock (Mission 3)\n');
fprintf('  S = Stop sensors (Mission 0)\n');
fprintf('  R = Resume sensors (Mission 1)\n');
fprintf('  Q = Quit\n\n');

% ===== L-SHAPED ROOM GEOMETRY =====
INCH = 0.0254;
MAIN_WIDTH  = 50 * INCH;
MAIN_HEIGHT = 30 * INCH;
EXT_WIDTH   = 36 * INCH;
EXT_HEIGHT  = 22 * INCH;
TOTAL_WIDTH  = MAIN_WIDTH;
TOTAL_HEIGHT = MAIN_HEIGHT + EXT_HEIGHT;

% ===== SINGLE LANDMARK =====
LANDMARK = [0.0, 0.0];

% ===== ROBOT PHYSICAL PARAMETERS =====
US_OFFSET_X = 0.05;
US_OFFSET_Y = 0.0;

% ================= CONFIG =================
N_MIN = 500;
N_MAX = 2000;
SENSOR_NOISE_STD = 0.03;
PROCESS_NOISE    = 0.08;
HEADING_NOISE_STD = 0.05;
RESAMPLE_THRESH_RATIO = 0.5;
RANDOM_INJECT    = 0.03;
CONVERGENCE_STD  = 0.4;

% ================= INIT =================
N = N_MAX;
particles.x = zeros(1, N);
particles.y = zeros(1, N);
particles.theta = (rand(1,N) - 0.5) * 2 * pi;

for i = 1:N
    valid = false;
    while ~valid
        x_test = rand * TOTAL_WIDTH;
        y_test = rand * TOTAL_HEIGHT;
        in_main = (y_test <= MAIN_HEIGHT);
        in_ext  = (y_test > MAIN_HEIGHT) && (x_test <= EXT_WIDTH);
        valid = in_main || in_ext;
    end
    particles.x(i) = x_test;
    particles.y(i) = y_test;
end
particles.w = ones(1,N) / N;

figure('Position',[100 100 1000 900], 'KeyPressFcn', @keyPressCallback);
trail_est = [];
prev_heading = [];

global mission_key;
mission_key = '';

fprintf('🚀 FILTER RUNNING - Click figure window to enable keyboard controls\n\n');

% Track consecutive read failures
consecutiveFailures = 0;
maxConsecutiveFailures = 50;

while true
    % ===== CHECK FOR KEYBOARD COMMANDS =====
    if ~isempty(mission_key)
        switch mission_key
            case 'd'
                writeline(s, "2");
                fprintf('>>> Mission 2 (DOCK) sent\n');
            case 'u'
                writeline(s, "3");
                fprintf('>>> Mission 3 (UNDOCK) sent\n');
            case 's'
                writeline(s, "0");
                fprintf('>>> Mission 0 (STOP) sent\n');
            case 'r'
                writeline(s, "1");
                fprintf('>>> Mission 1 (SENSORS) sent\n');
                consecutiveFailures = 0;
            case 'q'
                fprintf('>>> Quitting...\n');
                writeline(s, "0");
                pause(0.5);
                close all;
                clear s;
                return;
        end
        mission_key = '';
    end
    
    % ===== READ SENSOR DATA =====
    if s.NumBytesAvailable > 0
        try
            line = readline(s);
            
            if isempty(line) || strlength(line) == 0
                pause(0.01);
                continue;
            end
            
            if startsWith(line, "#")
                fprintf('%s\n', line);
                continue;
            end
            
            data = sscanf(line, "%f,%f,%f,%f,%f,%lu");
            if length(data) ~= 6
                continue;
            end
            
            consecutiveFailures = 0;
            
            heading = data(1);
            accel_x = data(2);
            accel_y = data(3);
            gyro_z  = data(4);
            us_cm   = data(5);
            
            tstamp  = data(6);
            
      
            us_m = us_cm / 100.0;
            % DEBUG: Print what we're receiving
            fprintf('Data: H=%.2f | US=%.1fcm (%.2fm) | t=%lu\n', heading, us_cm, us_m, tstamp);
            
           
            if us_m < 0.02 || us_m > 4.0
                fprintf('  -> REJECTED (out of range)\n');
                continue;
            end
            if us_m < 0.02 || us_m > 4.0
                continue;
            end

            % ===== PREDICT =====
            N_current = length(particles.x);
            particles.x = particles.x + randn(1,N_current)*PROCESS_NOISE;
            particles.y = particles.y + randn(1,N_current)*PROCESS_NOISE;
            particles.theta = particles.theta + randn(1,N_current)*0.1;

            in_main = particles.y <= MAIN_HEIGHT;
            in_ext  = (particles.y > MAIN_HEIGHT) & (particles.x <= EXT_WIDTH);
            valid   = in_main | in_ext;
            particles.w(~valid) = 0;

            % ===== UPDATE =====
            sensor_x = particles.x + US_OFFSET_X * cos(particles.theta);
            sensor_y = particles.y + US_OFFSET_Y * sin(particles.theta);
            
            pred_dist = sqrt((sensor_x - LANDMARK(1)).^2 + (sensor_y - LANDMARK(2)).^2);
            
            log_w = log(particles.w + 1e-300);
            log_w = log_w - 0.5*((pred_dist - us_m)/SENSOR_NOISE_STD).^2;
            
            if ~isempty(prev_heading)
                heading_innov = angdiff(particles.theta, heading);
                log_w = log_w - 0.5 * (heading_innov / HEADING_NOISE_STD).^2;
            end
            prev_heading = heading;
            
            log_w = log_w - max(log_w);
            particles.w = exp(log_w);
            particles.w = particles.w / sum(particles.w);

            % ===== ADAPTIVE PARTICLE COUNT =====
            spread = sqrt(var(particles.x) + var(particles.y));
            N_target = round(N_MIN + (N_MAX - N_MIN) * min(spread / 0.5, 1));
            
            n_inject = round(N_current * RANDOM_INJECT);
            [~, worst_idx] = mink(particles.w, n_inject);
            for i = 1:n_inject
                valid = false;
                while ~valid
                    x_test = rand * TOTAL_WIDTH;
                    y_test = rand * TOTAL_HEIGHT;
                    in_main = (y_test <= MAIN_HEIGHT);
                    in_ext  = (y_test > MAIN_HEIGHT) && (x_test <= EXT_WIDTH);
                    valid = in_main || in_ext;
                end
                idx = worst_idx(i);
                particles.x(idx) = x_test;
                particles.y(idx) = y_test;
                particles.theta(idx) = (rand - 0.5) * 2 * pi;
                particles.w(idx) = 1/N_current;
            end
            particles.w = particles.w / sum(particles.w);

            % ===== RESAMPLE =====
            Neff = 1/sum(particles.w.^2);
            if Neff < N_target * RESAMPLE_THRESH_RATIO
                if N_current < N_target
                    extra = N_target - N_current;
                    cum_w = cumsum(particles.w);
                    idx_extra = zeros(1, extra);
                    for k = 1:extra
                        r = rand();
                        idx_extra(k) = find(cum_w >= r, 1, 'first');
                    end
                    particles.x = [particles.x, particles.x(idx_extra) + randn(1,extra)*0.02];
                    particles.y = [particles.y, particles.y(idx_extra) + randn(1,extra)*0.02];
                    particles.theta = [particles.theta, particles.theta(idx_extra) + randn(1,extra)*0.05];
                    particles.w = ones(1, N_target) / N_target;
                elseif N_current > N_target
                    particles = systematic_resample(particles, N_target);
                else
                    particles = systematic_resample(particles, N_target);
                end
            end

            % ===== ESTIMATE =====
            spread = sqrt(var(particles.x) + var(particles.y));
            converged = spread < CONVERGENCE_STD;
            
            if converged
                est_x = sum(particles.x .* particles.w);
                est_y = sum(particles.y .* particles.w);
                est_theta = atan2(sum(sin(particles.theta).*particles.w), ...
                                  sum(cos(particles.theta).*particles.w));
            else
                [est_x, est_y] = densest_cluster(particles);
                est_theta = heading;
            end

            % ===== PLOT =====
            trail_est(end+1,:) = [est_x, est_y];

            clf; hold on;
            
            rectangle('Position',[0, 0, MAIN_WIDTH, MAIN_HEIGHT], ...
                      'EdgeColor','k', 'LineWidth', 2.5);
            rectangle('Position',[0, MAIN_HEIGHT, EXT_WIDTH, EXT_HEIGHT], ...
                      'EdgeColor','k', 'LineWidth', 2.5);
            patch([EXT_WIDTH, MAIN_WIDTH, MAIN_WIDTH, EXT_WIDTH], ...
                  [MAIN_HEIGHT, MAIN_HEIGHT, TOTAL_HEIGHT, TOTAL_HEIGHT], ...
                  [0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha',0.3);
            
            scatter(particles.x, particles.y, 8, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
            
            if size(trail_est,1)>1
                plot(trail_est(:,1), trail_est(:,2), 'r-', 'LineWidth', 2);
            end
            
            scatter(est_x, est_y, 200, 'r', 'filled', 'MarkerEdgeColor','k', 'LineWidth', 2);
            quiver(est_x, est_y, 0.15*cos(est_theta), 0.15*sin(est_theta), ...
                   0, 'r', 'LineWidth', 3, 'MaxHeadSize', 1.5);
            
            scatter(LANDMARK(1), LANDMARK(2), 300, 'k', 's', 'filled', 'LineWidth', 2);
            text(LANDMARK(1)+0.05, LANDMARK(2)-0.08, 'Landmark', 'FontSize', 10, 'FontWeight', 'bold');

            xlim([-0.05 TOTAL_WIDTH+0.05]); 
            ylim([-0.05 TOTAL_HEIGHT+0.05]); 
            grid on; axis equal;
            
            N_current = length(particles.x);
            status = "CONVERGED"; if ~converged, status = "searching..."; end
            
            title(sprintf('Real-Time Particle Filter | %s | N=%d | spread=%.3f | US=%.2fm', ...
                          status, N_current, spread, us_m));
            xlabel('x (m)'); ylabel('y (m)');
            
            text(0.02, TOTAL_HEIGHT-0.05, {'D=Dock  U=Undock  S=Stop  R=Resume  Q=Quit'}, ...
                 'FontSize', 9, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', 'k');
            
            drawnow limitrate;
            
        catch ME
            fprintf('Parse error: %s\n', ME.message);
        end
    else
        consecutiveFailures = consecutiveFailures + 1;
        
        if consecutiveFailures > maxConsecutiveFailures
            fprintf('\n⚠️  WARNING: No sensor data for 5 seconds!\n');
            fprintf('   Press R to restart sensors or check ESP connections\n');
            consecutiveFailures = 0;
        end
        
        pause(0.1);
    end
end

% ===== KEYBOARD CALLBACK =====
function keyPressCallback(~, event)
    global mission_key;
    mission_key = lower(event.Character);
end

% ===== HELPERS =====
function delta = angdiff(a, b)
    delta = mod(a - b + pi, 2*pi) - pi;
end

function [mx, my] = densest_cluster(particles)
    nbins = 20;
    edges_x = linspace(min(particles.x), max(particles.x), nbins+1);
    edges_y = linspace(min(particles.y), max(particles.y), nbins+1);
    H = histcounts2(particles.x, particles.y, edges_x, edges_y);
    [~, idx] = max(H(:));
    [ix, iy] = ind2sub(size(H), idx);
    in_bin = particles.x >= edges_x(ix) & particles.x < edges_x(ix+1) & ...
             particles.y >= edges_y(iy) & particles.y < edges_y(iy+1);
    if any(in_bin)
        mx = mean(particles.x(in_bin));
        my = mean(particles.y(in_bin));
    else
        mx = (edges_x(ix)+edges_x(ix+1))/2;
        my = (edges_y(iy)+edges_y(iy+1))/2;
    end
end

function particles = systematic_resample(particles, N)
    positions = ((0:N-1) + rand) / N;
    cum_w = cumsum(particles.w);
    indices = zeros(1,N);
    i=1; j=1;
    while i <= N
        if positions(i) < cum_w(j), indices(i)=j; i=i+1;
        else, j=j+1; end
    end
    particles.x = particles.x(indices) + randn(1,N) * 0.01;
    particles.y = particles.y(indices) + randn(1,N) * 0.01;
    particles.theta = particles.theta(indices) + randn(1,N) * 0.02;
    particles.w = ones(1,N) / N;
end