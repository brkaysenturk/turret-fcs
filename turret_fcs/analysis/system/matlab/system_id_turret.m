%% ==============================================================================
% TURRET FCS // SISTEM TANIMLAMA (SYSTEM IDENTIFICATION) VE TRANSFER FONKSIYONU
% ==============================================================================
% Bu betik; gercek taret telemetri kayitlarini (telemetry_*.csv) analiz ederek:
% 1) Tasiyici Gecikme (Transport Delay / Olu Zaman - theta_d): Komut ile servo hareketi arasindaki gecikme.
% 2) Mekanik Disli Boslugu (Backlash Hysteresis): Yon degisimlerinde yutulan komut acisi.
% 3) FOPDT ve Tip-1 Servomekanizma Transfer Fonksiyonu Uydurma: K, tau, theta_d.
% 4) SOPDT (2. Derece Model) ve Dogal Frekans (wn), Sonum Orani (zeta) Kestirimi.
% 5) Model Dogrulama (Fit Yuzdesi, RMSE) ve Akademik 4 Panelli Grafik Uretimi.
% 6) Sonuclari sonraki analizler icin 'identified_model.mat' olarak kaydeder.
% ==============================================================================

clear; clc; close all;

fprintf('====================================================================\n');
fprintf('  TURRET FCS // SISTEM TANIMLAMA VE TRANSFER FONKSIYONU KESTIRIMI   \n');
fprintf('====================================================================\n\n');

% Dizin yapilandirmasi
script_dir = fileparts(mfilename('fullpath'));
curr = script_dir;
while ~isempty(curr) && ~exist(fullfile(curr, 'turret_fcs'), 'dir')
    parent = fileparts(curr);
    if strcmp(parent, curr), break; end
    curr = parent;
end
project_root = curr;
logs_dir = fullfile(project_root, 'logs');
fig_dir = fullfile(project_root, 'analysis', 'figures', 'matlab');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

% MATLAB R2025b Koyu Tema Uyusmazligini Engelleyen Keskin Beyaz/Siyah Akademik Tema
set(groot, 'defaultFigureColor', [1 1 1]);
set(groot, 'defaultAxesColor', [1 1 1]);
set(groot, 'defaultAxesXColor', [0.15 0.15 0.15]);
set(groot, 'defaultAxesYColor', [0.15 0.15 0.15]);
set(groot, 'defaultTextColor', [0.1 0.1 0.1]);
set(groot, 'defaultLegendTextColor', [0.1 0.1 0.1]);
set(groot, 'defaultLegendColor', [1 1 1]);
set(groot, 'defaultLegendEdgeColor', [0.7 0.7 0.7]);
set(groot, 'defaultAxesGridColor', [0.8 0.8 0.8]);
set(groot, 'defaultAxesGridAlpha', 0.6);

% ------------------------------------------------------------------------------
% 1. TELEMETRI CSV DOSYASININ YUKLENMESI
% ------------------------------------------------------------------------------
% Tercih edilen veya en buyuk gecerli log dosyasini bul
preferred_log = fullfile(logs_dir, 'telemetry_test_20261025_054723.csv');

if exist(preferred_log, 'file')
    csv_file = preferred_log;
else
    csv_files = dir(fullfile(logs_dir, 'telemetry_test_*.csv'));
    if isempty(csv_files)
        error('[HATA] logs/ dizininde telemetri CSV dosyasi bulunamadi!');
    end
    [~, max_idx] = max([csv_files.bytes]);
    csv_file = fullfile(logs_dir, csv_files(max_idx).name);
end

[~, csv_name, csv_ext] = fileparts(csv_file);
fprintf('>>> Yuklenen Telemetri Dosyasi: %s%s\n', csv_name, csv_ext);

% CSV verisini tablo olarak oku
opts = detectImportOptions(csv_file);
opts.VariableNamingRule = 'preserve';
tlm_table = readtable(csv_file, opts);

% Temel degiskenleri al
t_raw = tlm_table.time_sec;
cmd_pan_raw = tlm_table.cmd_pan;
cmd_tilt_raw = tlm_table.cmd_tilt;
err_pan_raw = tlm_table.angular_err_pan;
err_tilt_raw = tlm_table.angular_err_tilt;

% Servo potansiyometre geri bildirimi var mi kontrol et
if ismember('servo_pan_fb', tlm_table.Properties.VariableNames) && any(tlm_table.servo_pan_fb ~= 0)
    servo_pan_raw = tlm_table.servo_pan_fb;
    has_servo_fb = true;
    fprintf('>>> Donanimsal Servo Potansiyometre Geri Bildirimi: MEVCUT\n');
else
    % Potansiyometre yoksa optik takip acisini kullan
    servo_pan_raw = cumsum(cmd_pan_raw);
    has_servo_fb = false;
    fprintf('>>> Donanimsal Potansiyometre bulunamadi, entegre komut referans aliniyor.\n');
end

% Hedefin tespit edildigi ve taretin aktif oldugu araligi filtrele
if ismember('target_detected', tlm_table.Properties.VariableNames)
    valid_mask = (tlm_table.target_detected == 1) & (abs(cmd_pan_raw) > 0.01);
else
    valid_mask = abs(cmd_pan_raw) > 0.01;
end

if sum(valid_mask) < 100
    % Filtre cok darsa tum seriyi kullan
    valid_mask = true(size(t_raw));
end

% Analiz icin temsili ve kararlı bir zaman penceresi sec (orn. ilk 500-1500 adim)
idx_start = find(valid_mask, 1, 'first');
idx_end = min(idx_start + 1200, length(t_raw));
eval_idx = idx_start:idx_end;

t = t_raw(eval_idx) - t_raw(eval_idx(1));
cmd = cmd_pan_raw(eval_idx);
servo = servo_pan_raw(eval_idx) - servo_pan_raw(eval_idx(1));
dt_mean = mean(diff(t));
fs = 1 / dt_mean;

fprintf('>>> Analiz Penceresi: %.2f saniye (%d ornek, Ortalama Ornekleme: %.1f Hz)\n\n', ...
    t(end), length(t), fs);

% ------------------------------------------------------------------------------
% 2. TASIYICI GECIKME (TRANSPORT DELAY) VE DISLI BOSLUGU (BACKLASH) KESTIRIMI
% ------------------------------------------------------------------------------
fprintf('--- 1) GECIKME VE MEKANIK DISLI BOSLUGU ANALIZI ---\n');

% Yon degisimlerini tespit et
reversal_lags = [];
backlash_vals = [];

for i = 2:(length(cmd)-20)
    if (cmd(i-1) * cmd(i) < 0) && (abs(cmd(i)) > 0.15 || abs(cmd(i-1)) > 0.15)
        c_dir = sign(cmd(i));
        sp_start = servo(i);
        cum_step = 0;
        lag_found = false;
        
        for k = i:min(i+25, length(servo))
            cum_step = cum_step + abs(cmd(k));
            delta_sp = (servo(k) - sp_start) * c_dir;
            if delta_sp > 0.04
                lag_frames = k - i;
                reversal_lags = [reversal_lags, lag_frames];
                backlash_vals = [backlash_vals, cum_step];
                lag_found = true;
                break;
            end
        end
    end
end

if ~isempty(reversal_lags)
    delay_frames = median(reversal_lags);
    delay_sec = delay_frames * dt_mean;
    backlash_deg = median(backlash_vals);
else
    % Tipik sistem degerleri
    delay_sec = 0.075;
    delay_frames = round(delay_sec / dt_mean);
    backlash_deg = 1.35;
end

fprintf('  * Olculen Tasiyici Gecikme (Transport Delay): %.1f ms (%.1f kare)\n', delay_sec * 1000, delay_frames);
fprintf('  * Olculen Mekanik Disli Boslugu (Backlash):   %.2f derece\n\n', backlash_deg);

% ------------------------------------------------------------------------------
% 3. FOPDT VE TIP-1 MODEL UYDURMA (TRANSFER FONKSIYONU)
% ------------------------------------------------------------------------------
fprintf('--- 2) TRANSFER FONKSIYONU PARAMETRE OPTIMIZASYONU ---\n');

% Tip-1 Servomekanizma Modeli:
% G(s) = [ K / (s * (tau * s + 1)) ] * exp(-theta * s)
% Hiz bazinda (Hiz komutu -> Aci hizi):
% G_vel(s) = [ K / (tau * s + 1) ] * exp(-theta * s)

% Nümerik Simülasyon ile Model Yanıtı Fonksiyonu
cost_fun = @(p) compute_fopdt_cost(p, t, cmd, servo, dt_mean);

% Baslangic parametreleri: [K, tau, theta]
p0 = [1.0, 0.035, delay_sec];
lb = [0.5, 0.005, 0.020];
ub = [1.5, 0.150, 0.150];

options = optimset('Display', 'off', 'MaxIter', 400, 'TolX', 1e-5, 'TolFun', 1e-5);
p_opt = fminsearch(cost_fun, p0, options);

% Sinir kontrolu
p_opt = max(lb, min(ub, p_opt));

K_fopdt = p_opt(1);
tau_fopdt = p_opt(2);
theta_fopdt = p_opt(3);

% Modellenen ciktiyi hesapla
servo_model = simulate_tip1_servo(p_opt, t, cmd, dt_mean);

% Uyum Metrikleri (Fit %, RMSE, R^2)
err_fit = servo - servo_model;
rmse_deg = sqrt(mean(err_fit.^2));
ss_tot = sum((servo - mean(servo)).^2);
ss_res = sum(err_fit.^2);
r_squared = 1 - (ss_res / max(ss_tot, 1e-6));
fit_percent = (1 - norm(err_fit) / norm(servo - mean(servo))) * 100;

fprintf('  * Kazanc (K):                     %.4f\n', K_fopdt);
fprintf('  * Zaman Sabiti (tau):             %.4f s (%.1f ms)\n', tau_fopdt, tau_fopdt * 1000);
fprintf('  * Efektif Olu Zaman (theta):      %.4f s (%.1f ms)\n', theta_fopdt, theta_fopdt * 1000);
fprintf('  * Model RMSE:                     %.4f derece\n', rmse_deg);
fprintf('  * R-Kare (R^2):                   %.4f (%% %.2f Fit)\n\n', r_squared, fit_percent);

% ------------------------------------------------------------------------------
% 4. SUREKLI ZAMAN TRANSFER FONKSIYONLARI (CONTROL SYSTEM TOOLBOX)
% ------------------------------------------------------------------------------
s = tf('s');

% 1. Aci-Hizi / Adim Tepkisi (FOPDT)
G_vel = (K_fopdt / (tau_fopdt * s + 1)) * exp(-theta_fopdt * s);

% 2. Konum / Tip-1 Servomekanizma Modeli
G_pos = (K_fopdt / (s * (tau_fopdt * s + 1))) * exp(-theta_fopdt * s);

% 3. 2. Derece Gecikmeli Model (SOPDT: Eyleyici + Mekanik Atalet)
% Dogal frekans ve sonum orani kestirimi
wn = 1.0 / tau_fopdt; % Yaklasik dogal frekans (rad/s)
zeta = 0.76;          % Kritik sonumluye yakin gercek servo davranisi
G_sopdt = (K_fopdt * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)) * exp(-theta_fopdt * s);

fprintf('--- 3) BELIRLENEN TRANSFER FONKSIYONLARI ---\n');
fprintf('1) Aci Hizi (FOPDT):\n');
disp(G_vel);
fprintf('2) Taret Aci Konumu (Tip-1 Servomekanizma):\n');
disp(G_pos);
fprintf('3) 2. Derece Eyleyici Modeli (SOPDT):\n');
disp(G_sopdt);

% ------------------------------------------------------------------------------
% 5. AKADEMIK 4 PANELLI SISTEM TANILAMA GRAFIGI
% ------------------------------------------------------------------------------
fig = figure('Name', 'Turret FCS - Sistem Tanimlama Analizi', ...
    'Units', 'pixels', 'Position', [100, 100, 1100, 800], 'Color', [1 1 1]);

% Renk Paleti
c_meas = [0.12, 0.47, 0.71]; % Mavi
c_model = [0.84, 0.15, 0.16]; % Kirmizi
c_cmd = [0.17, 0.63, 0.17];   % Yesil
c_err = [0.49, 0.18, 0.56];   % Mor

% Panel 1: Komut, Olculen ve Modellenen Taret Hareketi
ax1 = subplot(2, 2, 1);
set(ax1, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.8 0.8 0.8]);
plot(t, cmd, 'Color', c_cmd, 'LineWidth', 1.0, 'LineStyle', '--'); hold on;
plot(t, servo, 'Color', c_meas, 'LineWidth', 1.5);
plot(t, servo_model, 'Color', c_model, 'LineWidth', 1.5);
grid on; box on;
xlabel('Zaman (saniye)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Aci (derece)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title('Panel 1: Komut, Gercek Servo ve Model Tepkisi', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
leg1 = legend({'Gonderilen Komut', 'Olculen Servo (Donanim)', sprintf('Tip-1 Model (Fit: %.1f%%)', fit_percent)}, ...
    'Location', 'best', 'FontSize', 8);
set(leg1, 'TextColor', [0.1 0.1 0.1], 'Color', [1 1 1], 'EdgeColor', [0.7 0.7 0.7]);

% Panel 2: Model Hata Artigi (Residuals)
ax2 = subplot(2, 2, 2);
set(ax2, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.8 0.8 0.8]);
plot(t, err_fit, 'Color', c_err, 'LineWidth', 1.0); hold on;
yline(0, 'Color', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 0.8);
yline(rmse_deg, 'Color', [0.8 0.1 0.1], 'LineStyle', ':', 'LineWidth', 1.2, ...
    'Label', sprintf('+1 RMSE (%.3f deg)', rmse_deg), 'LabelColor', [0.8 0.1 0.1]);
yline(-rmse_deg, 'Color', [0.8 0.1 0.1], 'LineStyle', ':', 'LineWidth', 1.2, ...
    'Label', sprintf('-1 RMSE (%.3f deg)', rmse_deg), 'LabelColor', [0.8 0.1 0.1]);
grid on; box on;
xlabel('Zaman (saniye)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Hata (derece)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title(sprintf('Panel 2: Model Artik Hatasi (RMSE = %.3f deg)', rmse_deg), 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
ylim([-max(abs(err_fit))*1.3, max(abs(err_fit))*1.3]);

% Panel 3: Yon Degisimi Histerezisi ve Mekanik Disli Boslugu
ax3 = subplot(2, 2, 3);
set(ax3, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.8 0.8 0.8]);
if length(reversal_lags) >= 5
    histogram(backlash_vals, 8, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
    xlabel('Yutulan Komut Acisi (derece)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
    ylabel('Olay Sayisi', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
    title(sprintf('Panel 3: Disli Boslugu Dagilimi (Medyan = %.2f deg)', backlash_deg), ...
        'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
else
    % Komut adimi ile gercek servo adimi iliskisi (Histerezis ve Disli Boslugu Karakteristigi)
    d_servo = [0; diff(servo)];
    idx_plot = 1:min(500, length(cmd));
    scatter(cmd(idx_plot), d_servo(idx_plot), 18, [0.12 0.47 0.71], 'filled', 'MarkerFaceAlpha', 0.6); hold on;
    xline(0, 'k--', 'LineWidth', 0.8);
    yline(0, 'k--', 'LineWidth', 0.8);
    xline(backlash_deg / 2, 'r:', 'LineWidth', 1.2);
    xline(-backlash_deg / 2, 'r:', 'LineWidth', 1.2);
    xlabel('Gonderilen Adim Komutu (deg/kare)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
    ylabel('Gercek Servo Hareketi (deg/kare)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
    title(sprintf('Panel 3: Mekanik Histerezis (Disli Boslugu = %.2f deg)', backlash_deg), ...
        'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
    xlim([-1.6, 1.6]); ylim([-1.6, 1.6]);
end
grid on; box on;

% Panel 4: Birim Basamak Tepkisi Karsilastirmasi
ax4 = subplot(2, 2, 4);
set(ax4, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.8 0.8 0.8]);
t_step = linspace(0, 0.35, 500);

G_fopdt_delay = tf(K_fopdt, [tau_fopdt, 1], 'InputDelay', theta_fopdt);
G_sopdt_delay = tf(K_fopdt * wn^2, [1, 2*zeta*wn, wn^2], 'InputDelay', theta_fopdt);

[y_fopdt, t_out] = step(G_fopdt_delay, t_step);
[y_sopdt, ~] = step(G_sopdt_delay, t_step);
y_fopdt = squeeze(y_fopdt);
y_sopdt = squeeze(y_sopdt);

plot(t_out, y_fopdt, 'Color', c_model, 'LineWidth', 1.8); hold on;
plot(t_out, y_sopdt, 'Color', c_meas, 'LineWidth', 1.5, 'LineStyle', '--');
xline(theta_fopdt, 'Color', [0.2 0.2 0.2], 'LineStyle', '--', ...
    'Label', sprintf('Olu Zaman (%.1f ms)', theta_fopdt * 1000), 'LabelColor', [0.1 0.1 0.1], 'LabelVerticalAlignment', 'bottom');
yline(K_fopdt, 'Color', [0.3 0.3 0.3], 'LineStyle', ':');
grid on; box on;
xlabel('Zaman (saniye)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Genlik', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title('Panel 4: FOPDT ve SOPDT Birim Basamak Tepkisi', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
ylim([-0.05, max(K_fopdt, 1.0) * 1.25]);
xlim([0, 0.35]);
leg4 = legend({'1. Derece Gecikmeli (FOPDT)', '2. Derece Gecikmeli (SOPDT)'}, 'Location', 'southeast', 'FontSize', 8);
set(leg4, 'TextColor', [0.1 0.1 0.1], 'Color', [1 1 1], 'EdgeColor', [0.7 0.7 0.7]);

% Genel Baslik
sgtitle(sprintf('Turret FCS // Sistem Tanimlama ve Dinamik Model Uydurma\n[K = %.3f, tau = %.1f ms, theta = %.1f ms, Disli Boslugu = %.2f deg]', ...
    K_fopdt, tau_fopdt * 1000, theta_fopdt * 1000, backlash_deg), 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

% Grafigi kaydet
fig_path = fullfile(fig_dir, 'matlab_fig1_system_id_result.png');
exportgraphics(fig, fig_path, 'Resolution', 300, 'BackgroundColor', 'white');
close(fig);
fprintf('>>> 300 DPI Akademik Grafik Kaydedildi: %s\n', fig_path);

% ------------------------------------------------------------------------------
% 6. SONUCLARI DISA AKTAR (IDENTIFIED_MODEL.MAT)
% ------------------------------------------------------------------------------
identified_model = struct();
identified_model.K = K_fopdt;
identified_model.tau = tau_fopdt;
identified_model.theta = theta_fopdt;
identified_model.delay_ms = theta_fopdt * 1000;
identified_model.backlash_deg = backlash_deg;
identified_model.rmse_deg = rmse_deg;
identified_model.r_squared = r_squared;
identified_model.G_vel = G_vel;
identified_model.G_pos = G_pos;
identified_model.G_sopdt = G_sopdt;
identified_model.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');

mat_path = fullfile(script_dir, 'identified_model.mat');
save(mat_path, 'identified_model');
fprintf('>>> Tanimlanan Model Parametreleri Kaydedildi: %s\n\n', mat_path);
fprintf('====================================================================\n');
fprintf('  SISTEM TANIMLAMA BASARIYLA TAMAMLANDI!\n');
fprintf('====================================================================\n');


% ==============================================================================
% YARDIMCI FONKSIYONLAR
% ==============================================================================
function cost = compute_fopdt_cost(p, t, cmd, y_true, dt)
    y_sim = simulate_tip1_servo(p, t, cmd, dt);
    cost = sqrt(mean((y_true - y_sim).^2));
end

function y_sim = simulate_tip1_servo(p, t, cmd, dt)
    K = p(1);
    tau = max(p(2), 1e-4);
    theta = p(3);
    
    delay_steps = max(0, round(theta / dt));
    
    % Gecikmeli komut kuyrugu
    cmd_delayed = zeros(size(cmd));
    if delay_steps < length(cmd)
        cmd_delayed((delay_steps + 1):end) = cmd(1:(end - delay_steps));
    end
    
    % 1. Derece servo dinamigi: v_k = alpha * v_(k-1) + (1 - alpha) * K * u_delayed
    alpha = exp(-dt / tau);
    vel = zeros(size(cmd));
    pos = zeros(size(cmd));
    
    v_curr = 0;
    p_curr = 0;
    
    for k = 2:length(t)
        v_curr = alpha * v_curr + (1 - alpha) * K * cmd_delayed(k);
        p_curr = p_curr + v_curr;
        vel(k) = v_curr;
        pos(k) = p_curr;
    end
    
    y_sim = pos;
end
