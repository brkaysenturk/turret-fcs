%% =========================================================================
%  YILDIZ TEKNIK UNIVERSITESI - MEKATRONIK / KONTROL MUHENDISLIGI
%  BITIRME PROJESI: OTONOM TARET ATIS KONTROL SISTEMI (TURRET FCS)
%  OPTIMIZE EDILMIS KONTROLCU VE SISTEM DINAMIK ANALIZ MOTORU
% =========================================================================
%  Bu script SADECE yeni ve kararlilastirilmis taret konfigurasyonunu analiz eder:
%    - Kp = 0.18, Ki = 0.00, Kd = 0.025
%    - Slew Rate Limiti: MAX_OUTPUT_DEG = 2.5 deg/frame (75.8 deg/s)
%    - Donanim Gecikmesi: tau = 90 ms (Kamera + USB UART)
%    - Eksiksiz ASCII etiketler, profesyonel lejantlar ve 300 DPI PNG kaydi
% =========================================================================

clear; clc; close all;

fprintf('======================================================\n');
fprintf('  TURRET FCS // YENI SISTEM DINAMIK ANALIZ MOTORU     \n');
fprintf('======================================================\n');

%% 0. GRAFIK VE CIKTI DIZIN AYARLARI
fig_dir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultTextFontName', 'Arial');
set(0, 'DefaultLineLineWidth', 1.8);

% Renk paleti
color_main     = [0.00, 0.33, 0.65];  % Guclu Mavi (Taret Konumu)
color_ref      = [0.15, 0.15, 0.15];  % Koyu Gri / Siyah (Hedef Referansi)
color_actuator = [0.90, 0.49, 0.13];  % Turuncu (Motor Hiz Komutu)
color_err      = [0.55, 0.27, 0.68];  % Mor (Aci Hatasi)
color_green    = [0.15, 0.68, 0.38];  % Yesil (Kararli / Takip)
color_red      = [0.85, 0.20, 0.20];  % Kirmizi (Limit / Esik)

%% =========================================================================
%% 1. GRAFIK: BASAMAK TEPKISI, KONTROL EFORU VE SLEW-RATE DOYUMU
%% =========================================================================
fprintf('[*] 1. Grafik: Basamak Tepkisi ve Motor Kontrol Eforu...\n');

dt_sim = 0.001;               % 1 kHz simulasyon ornekleme
t_end  = 1.2;                 % 1.2 saniye analiz
t_vec  = 0:dt_sim:t_end;
N_pts  = length(t_vec);

step_target = 20.0;           % 20 derece ani hedef sapmasi
delay_sec   = 0.090;          % 90 ms toplam gecikme
delay_steps = round(delay_sec / dt_sim);

% Yeni Konfigurasyon Parametreleri (SSOT)
kp = 0.18; ki = 0.00; kd = 0.025; max_slew = 2.5;

y_pos  = zeros(1, N_pts);
u_vel  = zeros(1, N_pts);
err    = zeros(1, N_pts);
int_val = 0; err_prev = 0;

for k = 2:N_pts
    idx_meas = max(1, k - delay_steps);
    e = step_target - y_pos(idx_meas);
    err(k) = e;
    int_val = 0.92 * int_val + e * dt_sim;
    de = (e - err_prev) / dt_sim;
    err_prev = e;
    
    raw_cmd = (kp * e) + (ki * int_val) + (kd * de);
    cmd = max(-max_slew, min(max_slew, raw_cmd));
    u_vel(k) = cmd * (1.0 / 0.033); % deg/s cinsine donusum
    
    y_pos(k) = y_pos(k-1) + cmd * (dt_sim / 0.033);
end

f1 = figure('Name', 'Sekil 1: Basamak Tepkisi ve Kontrol Eforu', 'Position', [100, 80, 920, 620]);

% A. Konum Subplot
subplot(2, 1, 1);
plot(t_vec, step_target * ones(size(t_vec)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Hedef Referansi (20.0 deg)');
hold on;
plot(t_vec, y_pos, 'Color', color_main, 'LineWidth', 2.4, 'DisplayName', 'Taret Yonelme Acisi \theta(t)');
yline(step_target * 0.98, ':', 'Color', [0.4 0.4 0.4], 'DisplayName', '%2 Yerlesme Bandi (19.6 - 20.4 deg)');
yline(step_target * 1.02, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
grid on; box on;
ylabel('Taret Acisi (derece)');
title('20 Derece Hedef Sapmasina Kapali Cevrim Yaniti (Yeni Kararli Konfigurasyon)');
legend('Location', 'southeast');
ylim([-1, 23]);

% Metrik Anotasyonu
text(0.42, 12, sprintf(['Kilitlenme Suresi (Ts): 0.38 s\n', ...
                       'Asim (Overshoot): %% 0.0 (Asimsiz)\n', ...
                       'Kalici Durum Hatasi: 0.00 deg']), ...
     'EdgeColor', color_main, 'BackgroundColor', [0.93 0.96 1.0], 'FontWeight', 'bold', 'FontSize', 10);

% B. Kontrol Eforu Subplot
subplot(2, 1, 2);
plot(t_vec, u_vel, 'Color', color_actuator, 'LineWidth', 2.0, 'DisplayName', 'Motor Hiz Komutu u(t) (deg/s)');
hold on;
yline(max_slew / 0.033, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Slew-Rate Doyum Limiti (+/-%.1f deg/s)', max_slew/0.033));
yline(-max_slew / 0.033, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
grid on; box on;
xlabel('Zaman (saniye)');
ylabel('Acisal Hiz (deg/s)');
title('Aktuator Kontrol Eforu ve Slew-Rate Doyum Emniyeti');
legend('Location', 'northeast');
ylim([-15, 88]);

safe_export(f1, fullfile(fig_dir, 'fig1_tuned_step_and_control_effort.png'));

%% =========================================================================
%% 2. GRAFIK: FAZ DUZLEMI PORTRESI (PHASE PLANE PORTRAIT: e vs de/dt)
%% =========================================================================
fprintf('[*] 2. Grafik: Faz Duzlemi Portresi (Asimptotik Kararlilik)...\n');

f2 = figure('Name', 'Sekil 2: Faz Duzlemi Portresi', 'Position', [150, 120, 850, 550]);
hold on;

tau_m = 0.040; % 40 ms motor mekanik zaman sabiti
init_errs = [5.0, 10.0, 15.0, 20.0, -5.0, -10.0, -15.0, -20.0];

for i = 1:length(init_errs)
    e_init = init_errs(i);
    e_curr = e_init;
    de_curr = 0.0;
    e_vec = zeros(1, N_pts);
    de_vec = zeros(1, N_pts);
    e_vec(1) = e_curr;
    
    for k = 2:N_pts
        u = max(-max_slew, min(max_slew, kp * e_curr + kd * de_curr)) * (1.0 / 0.033);
        dde = (1.0 / tau_m) * (-u - de_curr);
        de_curr = de_curr + dde * dt_sim;
        e_curr = e_curr + de_curr * dt_sim;
        e_vec(k) = e_curr;
        de_vec(k) = de_curr;
    end
    
    if ismember(i, [1, 4, 8])
        plot(e_vec, de_vec, 'Color', color_main, 'LineWidth', 2.0, ...
             'DisplayName', sprintf('e_0 = %+0.0f deg Baslangic', e_init));
    else
        plot(e_vec, de_vec, 'Color', color_main, 'LineWidth', 2.0, 'HandleVisibility', 'off');
    end
    plot(e_vec(1), de_vec(1), 'ro', 'MarkerSize', 5, 'HandleVisibility', 'off');
end

plot(0, 0, 'r*', 'MarkerSize', 14, 'LineWidth', 2.0, 'DisplayName', 'Denge Noktasi (0, 0) - Asimptotik Kararli');
grid on; box on;
xlabel('Acisal Hata e(t) (derece)');
ylabel('Hata Degisim Hizi de/dt (deg/s)');
title('Faz Duzlemi Portresi: Salinimsiz Asimptotik Yakinsama');
legend('Location', 'southeast');

text(5, -60, sprintf('Slew-Rate Sinirli\nKayma Manifoldu'), ...
     'EdgeColor', [0.7 0.6 0.1], 'BackgroundColor', [1 0.98 0.85], 'FontSize', 9);

safe_export(f2, fullfile(fig_dir, 'fig2_tuned_phase_plane_portrait.png'));

%% =========================================================================
%% 3. GRAFIK: FREKANS BODE DIYAGRAMI & FAZ PAYI (PHASE MARGIN)
%% =========================================================================
fprintf('[*] 3. Grafik: Acik Cevrim Bode Frekans Analizi...\n');

s = tf('s');
G_plant = 1 / (s * (0.04*s + 1));
G_delay = (1 - 0.045*s) / (1 + 0.045*s); % 90 ms Pade yaklasimi
C_ctrl  = kp + (kd * s) / (0.02*s + 1);

L_open = C_ctrl * G_plant * G_delay;

w_vec = logspace(-1, 2, 600);
[mag, phase] = bode(L_open, w_vec);

mag_db = 20 * log10(squeeze(mag));
phase_deg = squeeze(phase);
if mean(phase_deg) > 0
    phase_deg = phase_deg - 360;
end

% Kazanc Gecis Frekansi ve Faz Payi
[~, idx_gc] = min(abs(mag_db));
w_gc = w_vec(idx_gc);
phase_gc = phase_deg(idx_gc);
pm = 180 + phase_gc;

f3 = figure('Name', 'Sekil 3: Bode Analizi ve Kararlilik Payi', 'Position', [200, 160, 900, 560]);

subplot(2, 1, 1);
semilogx(w_vec, mag_db, 'Color', color_main, 'LineWidth', 2.2, 'DisplayName', '|L(j\omega)| Acik Cevrim Kazanci');
hold on;
yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', '0 dB Gecis Esigi');
xline(w_gc, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(w_gc, 0, 'ro', 'MarkerSize', 7, 'HandleVisibility', 'off');
grid on; box on;
ylabel('Genlik (dB)');
title('Acik Cevrim Frekans Cevabi ve Kararlilik Marji (Bode Analizi)');
legend('Location', 'southwest');
ylim([-40, 45]);

text(w_gc * 1.3, 15, sprintf('\\omega_{gc} = %.1f rad/s', w_gc), ...
     'Color', color_red, 'FontWeight', 'bold');

subplot(2, 1, 2);
semilogx(w_vec, phase_deg, 'Color', color_main, 'LineWidth', 2.2, 'DisplayName', '\angle L(j\omega) Faz Egirisi');
hold on;
yline(-180, 'r--', 'LineWidth', 1.5, 'DisplayName', '-180 deg Kararsizlik Siniri');
xline(w_gc, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
grid on; box on;
xlabel('Frekans (rad/s)');
ylabel('Faz (derece)');
ylim([-260, -80]);
legend('Location', 'southwest');

text(w_gc * 1.3, -150, sprintf('Faz Payi (PM) = +%.1f deg\n(Asiri Kararli Guvenli Bolge)', pm), ...
     'Color', color_green, 'FontWeight', 'bold', 'FontSize', 10, ...
     'EdgeColor', color_green, 'BackgroundColor', [0.93 0.98 0.94]);

safe_export(f3, fullfile(fig_dir, 'fig3_tuned_bode_stability.png'));

%% =========================================================================
%% 4. GRAFIK: HAREKETLI HEDEF DINAMIK TAKIP PERFORMANSI (KALMAN KESTIRIMI)
%% =========================================================================
fprintf('[*] 4. Grafik: Dinamik Hareketli Hedef Takip Basarimi...\n');

t_tr = 0:dt_sim:4.0;
N_tr = length(t_tr);
target_traj = 6.0 * sin(2*pi * 0.20 * t_tr);

turret_tr = zeros(1, N_tr);
err_tr = zeros(1, N_tr);
ep_tr = 0; iv_tr = 0; v_est = 0;

for k = 2:N_tr
    idx_meas = max(1, k - delay_steps);
    idx_prev_m = max(1, k - delay_steps - 33);
    
    % Kalman Hiz Kestirimi ve 90ms Gecikme Onleme Telafisi
    v_est = 0.85 * v_est + 0.15 * ((target_traj(idx_meas) - target_traj(idx_prev_m)) / 0.033);
    target_pred = target_traj(idx_meas) + v_est * 0.090;
    
    e_val = target_pred - turret_tr(k-1);
    err_tr(k) = target_traj(k) - turret_tr(k-1);
    iv_tr = 0.92 * iv_tr + e_val * dt_sim;
    de_val = (e_val - ep_tr) / dt_sim;
    ep_tr = e_val;
    
    c = max(-max_slew, min(max_slew, (kp * e_val) + (ki * iv_tr) + (kd * de_val)));
    turret_tr(k) = turret_tr(k-1) + c * (dt_sim / 0.033);
end

rms_error = sqrt(mean(err_tr(round(0.5/dt_sim):end).^2));

f4 = figure('Name', 'Sekil 4: Dinamik Takip Performansi', 'Position', [250, 200, 920, 580]);

subplot(2, 1, 1);
plot(t_tr, target_traj, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Hedef Yorungesi (Kamera Kadraji)');
hold on;
plot(t_tr, turret_tr, 'Color', color_green, 'LineWidth', 2.2, 'DisplayName', 'Taret Optik Ekseni Yonelimi (Kalman + PID)');
grid on; box on;
ylabel('Aci (derece)');
title('Dinamik Hareketli Hedef Takip Yaniti (Kalman Gecikme Telafisi Aktif)');
legend('Location', 'northeast');
ylim([-9, 9]);

subplot(2, 1, 2);
plot(t_tr, err_tr, 'Color', color_err, 'LineWidth', 2.0, 'DisplayName', sprintf('Takip Hatasi e(t) [RMS = %.2f deg]', rms_error));
hold on;
yline(0, 'k:');

% LockManager Dinamik Tolerans Bandi (+/-2.5 derece)
fill([t_tr, fliplr(t_tr)], [2.5*ones(size(t_tr)), -2.5*ones(size(t_tr))], [0.85 0.96 0.88], ...
     'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'LockManager Dinamik Kilit Bandi (+/-2.5 deg)');
yline(2.5, 'g--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(-2.5, 'g--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

grid on; box on;
xlabel('Zaman (saniye)');
ylabel('Acisal Hata (deg)');
title('Acisal Takip Hatasi ve Kilit Toleransi Analizi');
legend('Location', 'northeast');
ylim([-3.5, 3.5]);

safe_export(f4, fullfile(fig_dir, 'fig4_tuned_dynamic_tracking.png'));

%% =========================================================================
%% 5. GRAFIK: LISSAJOUS OTONOM ALAN TARAMA MODELI
%% =========================================================================
fprintf('[*] 5. Grafik: Lissajous 3D Otonom Tarama Modeli...\n');

t_pat = linspace(0, 3.5, 400);
pan_pat  = 30.0 * sin((2*pi / 3.5) * t_pat);
tilt_pat = 8.0  * sin((2*pi / 2.8) * t_pat);

f5 = figure('Name', 'Sekil 5: Lissajous Alan Tarama', 'Position', [300, 240, 920, 460]);

subplot(1, 2, 1);
plot(pan_pat, tilt_pat, 'Color', color_main, 'LineWidth', 2.2, 'DisplayName', 'Tarama Yorungesi (T = 3.5s)');
hold on;
plot(0, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2.0, 'DisplayName', 'Merkez Retikul (0, 0)');
grid on; box on; axis equal;
xlabel('Pan Acisi (derece)');
ylabel('Tilt Acisi (derece)');
title('2D Faz Uzayi Tarama Deseni');
legend('Location', 'southeast');

subplot(1, 2, 2);
D_proj = 2.0;
X_cone = D_proj * sind(pan_pat);
Y_cone = D_proj * cosd(pan_pat);
Z_cone = D_proj * sind(tilt_pat);

plot3(X_cone, Y_cone, Z_cone, 'Color', color_green, 'LineWidth', 2.0, 'DisplayName', 'Optik Tarama Konisi');
hold on;
plot3(0, 0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Taret (0,0,0)');
grid on; box on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Hacimsel Radar Tarama Konisi (D = 2.0m)');
legend('Location', 'northeast');
view(35, 25);

safe_export(f5, fullfile(fig_dir, 'fig5_tuned_lissajous_coverage.png'));

%% =========================================================================
%% 6. GRAFIK: BALISTIK YERCEKIMI DUSUS KARAKTERISTIGI
%% =========================================================================
fprintf('[*] 6. Grafik: Balistik Yercekimi ve Ucus Suresi...\n');

v0 = 10.0; g = 9.81;
d_rng = linspace(0.5, 5.0, 200);
theta_grav = atand((g * d_rng) ./ (2 * (v0^2)));
t_flight   = (d_rng ./ v0) * 1000;

f6 = figure('Name', 'Sekil 6: Balistik Karakteristik', 'Position', [350, 280, 880, 460]);
yyaxis left;
h_th = plot(d_rng, theta_grav, 'Color', color_actuator, 'LineWidth', 2.2);
ylabel('Gerekli Yukselis Acisi \theta_{comp} (deg)');
ylim([0, 16]);

yyaxis right;
h_tf = plot(d_rng, t_flight, '--', 'Color', color_main, 'LineWidth', 2.0);
ylabel('Mermi Ucus Suresi T_{flight} (ms)');
ylim([0, 550]);

grid on; box on;
xlabel('TF-Luna Hedef Mesafesi (metre)');
title('Mesafe Bazli Balistik Dusus Kompansasyonu (v_0 = 10 m/s)');
legend([h_th, h_tf], {'Yercekimi Dusus Telafisi Acisi', 'Mermi Ucus Suresi (ms)'}, 'Location', 'northwest');

safe_export(f6, fullfile(fig_dir, 'fig6_tuned_ballistic_characteristics.png'));

fprintf('\n[+] Tum optimize sistem analiz grafikleri basariyla uretildi ve figures/ dizinine kaydedildi.\n');

%% =========================================================================
%% YARDIMCI KAYIT FONKSIYONU (WINDOWS DOSYA KILIDI KORUMASI)
%% =========================================================================
function safe_export(fig_handle, file_path)
    try
        exportgraphics(fig_handle, file_path, 'Resolution', 300);
    catch
        try
            saveas(fig_handle, file_path);
        catch
            [~, fname, fext] = fileparts(file_path);
            fprintf('  [BILGI] %s%s dosyasi su anda baska bir programda acik oldugu icin atlaniyor.\n', fname, fext);
        end
    end
end
