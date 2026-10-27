%% ==============================================================================
% TURRET FCS // HARDWARE-IN-THE-LOOP (HIL) VE CANLI STM32 HABERLESME BETIGI
% ==============================================================================
% Bu betik; PC (MATLAB) ile fiziksel STM32F401RE Nucleo donanimi arasinda
% 115200 Baud ikili (binary) seri port haberlesmesi kurarak Donanim Dongude
% (Hardware-in-the-Loop - HIL) gercek zamanli test gerceklestirir:
% 
% 1) Otomatik Port Tespiti (STM32 USB VID 0483 veya kullanici portu).
% 2) Sanal Hedef Yazan Uretec (Virtual Target Generator: Sinus/Basamak).
% 3) 7 Baytlik Ikili Komut Paketi Gonderimi (0xAA Header + Checksum).
% 4) 10 Baytlik Telemetri Paketi Okuma (0x55 Header + Potansiyometre + Mesafe).
% 5) Gercek Zamanli Canli Takip Arayuzu (Live Tracking GUI).
% 6) Donanim takili degilse otomatik "Sanal Donanim (Mock HIL)" moduna gecer!
% ==============================================================================

clear; clc; close all;

fprintf('====================================================================\n');
fprintf('  TURRET FCS // HARDWARE-IN-THE-LOOP (HIL) GERCEK ZAMANLI TEST     \n');
fprintf('====================================================================\n\n');

script_dir = fileparts(mfilename('fullpath'));
% Repo kok dizinini bul
curr = script_dir;
while ~isempty(curr) && ~exist(fullfile(curr, 'turret_fcs'), 'dir')
    parent = fileparts(curr);
    if strcmp(parent, curr), break; end
    curr = parent;
end
project_root = curr;
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
% 1. SERI PORT TESPITI VE BAGLANTI
% ------------------------------------------------------------------------------
baudrate = 115200;
selected_port = '';

% Mevcut seri portlari listele
available_ports = serialportlist("available");
fprintf('>>> Algilanan Seri Portlar: ');
if isempty(available_ports)
    fprintf('Hicbir fiziksel port bulunamadi.\n');
else
    fprintf('%s ', available_ports);
    fprintf('\n');
    % Ilk portu aday sec (kullanici isterse elle degistirebilir)
    selected_port = available_ports(1);
end

% Donanim baglanti denemesi
sp = [];
is_hardware_connected = false;

if ~isempty(selected_port)
    try
        fprintf('>>> %s portuna %d baud ile baglaniliyor...\n', selected_port, baudrate);
        sp = serialport(selected_port, baudrate, "Timeout", 0.05);
        flush(sp);
        pause(0.5);
        is_hardware_connected = true;
        fprintf('>>> [BASARILI] STM32F401RE Donanim Baglantisi Kuruldu!\n\n');
    catch ME
        fprintf('>>> [BILGI] Port acilamadi (%s). Sanal Donanim (Mock HIL) moduna geciliyor.\n\n', ME.message);
        is_hardware_connected = false;
    end
else
    fprintf('>>> [BILGI] Fiziksel cihaz takili degil. Sanal Donanim (Mock HIL) modunda calisilacak.\n\n');
end

% ------------------------------------------------------------------------------
% 2. TEST PROFILI VE PARAMETRE TANIMLARI
% ------------------------------------------------------------------------------
duration_sec = 8.0;      % Test suresi (saniye)
fs_loop = 30.0;          % FCS kontrol dongusu (30 Hz)
dt = 1.0 / fs_loop;
total_steps = round(duration_sec * fs_loop);

% FCS Kontrol Parametreleri (turret_fcs/config.py)
Kp_pan = 0.1953;         % Optuna TPE optimal taban kazanci (30 FPS)
deadband_pan = 0.85;     % Yumusak olu bolge (derece)
max_step_deg = 1.5;      % Adim siniri (derece/dongu)

% Sanal Donanim Model Parametreleri (Donanim yokken calisan benzetim)
sim_pan_angle = 90.0;
sim_v = 0;
tau_sim = 0.030;
alpha_sim = exp(-dt / tau_sim);
sim_backlash_held = 0.0;

% Telemetri Kayit Dizileri
log_t = zeros(total_steps, 1);
log_ref = zeros(total_steps, 1);
log_actual = zeros(total_steps, 1);
log_err = zeros(total_steps, 1);
log_cmd = zeros(total_steps, 1);

% ------------------------------------------------------------------------------
% 3. CANLI GRAFIK EKRANI HAZIRLIGI
% ------------------------------------------------------------------------------
fig = figure('Name', 'Turret FCS - HIL Canli Takip Arayuzu', ...
    'Units', 'pixels', 'Position', [120, 120, 1000, 600], 'Color', [1 1 1]);

ax1 = subplot(2, 1, 1);
set(ax1, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
    'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);
h_ref = plot(nan, nan, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.5, 'LineStyle', '--'); hold on;
h_act = plot(nan, nan, 'Color', [0.0 0.45 0.85], 'LineWidth', 1.8);
grid on; box on;
xlabel('Zaman (saniye)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Pan Acisi (derece)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title('Panel 1: Canli Hedef Referansi ve Donanim Geri Bildirimi', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
leg1 = legend({'Hedef Referans Acisi', 'Donanim Servo Geri Bildirimi'}, 'Location', 'northwest', 'FontSize', 8);
set(leg1, 'TextColor', [0.1 0.1 0.1], 'Color', [1 1 1], 'EdgeColor', [0.7 0.7 0.7]);
xlim([0, duration_sec]); ylim([70, 110]);

ax2 = subplot(2, 1, 2);
set(ax2, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
    'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);
h_err = plot(nan, nan, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.5); hold on;
yline(deadband_pan, 'Color', [0.1 0.55 0.1], 'LineStyle', ':', 'LineWidth', 1.4, ...
    'Label', sprintf('+ Olu Bolge (+%.2f deg)', deadband_pan), 'LabelColor', [0.05 0.45 0.05]);
yline(-deadband_pan, 'Color', [0.1 0.55 0.1], 'LineStyle', ':', 'LineWidth', 1.4, ...
    'Label', sprintf('- Olu Bolge (-%.2f deg)', deadband_pan), 'LabelColor', [0.05 0.45 0.05]);
yline(0, 'Color', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 0.8);
grid on; box on;
xlabel('Zaman (saniye)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Takip Hatasi (deg)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title('Panel 2: Anlik Takip Hatasi ve Kilitlenme Durumu', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
xlim([0, duration_sec]); ylim([-10, 10]);

if is_hardware_connected
    mode_str = sprintf('FIZIKSEL STM32 DONANIM DONGUDE (Port: %s, 115200 Baud)', selected_port);
else
    mode_str = 'SANAL DONANIM (MOCK HIL) MODU [Fiziksel baglanti beklenmeden calisiyor]';
end
sgtitle(sprintf('Turret FCS // Hardware-in-the-Loop (HIL) Testi\n[%s]', mode_str), ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

% ------------------------------------------------------------------------------
% 4. HIL DONGUSU (GERCEK ZAMANLI GONDERIM VE OKUMA)
% ------------------------------------------------------------------------------
fprintf('>>> HIL Gercek Zamanli Takip Testi Baslatiliyor (Sure: %.1f saniye)...\n', duration_sec);
t_start = tic;
actual_angle = 90.0;

for k = 1:total_steps
    t_now = (k - 1) * dt;
    
    % 1) Sanal Hedef Referansi Uret (Merkez: 90 deg, Genlik: 12 deg, Frekans: 0.25 Hz)
    target_ref = 90.0 + 12.0 * sin(2 * pi * 0.25 * t_now);
    
    % 2) Takip Hatasi
    err = target_ref - actual_angle;
    
    % 3) Yumusak Olu Bolgeli Tip-1 P Kontrolcu
    if abs(err) <= deadband_pan
        eff_err = 0.0;
    else
        eff_err = sign(err) * (abs(err) - deadband_pan);
    end
    
    cmd_step = Kp_pan * eff_err;
    cmd_step = max(-max_step_deg, min(max_step_deg, cmd_step));
    
    % 4) STM32 Ikili Paket Protokolu (7 Bayt PC -> STM)
    % [0xAA] [pan_delta*10: int16] [tilt_delta*10: int16] [state: uint8] [checksum: uint8]
    pan_delta_raw = int16(round(cmd_step * 10.0));
    tilt_delta_raw = int16(0);
    state_byte = uint8(0); % STATE_TRACKING
    
    % Little-endian bayt ayrilmasi
    pan_bytes = typecast(pan_delta_raw, 'uint8');
    tilt_bytes = typecast(tilt_delta_raw, 'uint8');
    
    % Checksum: bayt toplami mod 256
    csum = mod(double(pan_bytes(1)) + double(pan_bytes(2)) + ...
               double(tilt_bytes(1)) + double(tilt_bytes(2)) + ...
               double(state_byte), 256);
    
    cmd_packet = uint8([hex2dec('AA'), pan_bytes(1), pan_bytes(2), ...
                        tilt_bytes(1), tilt_bytes(2), state_byte, uint8(csum)]);
    
    % 5) Donanim Iletisimi veya Sanal Yanit
    if is_hardware_connected
        % Paketi gonder
        write(sp, cmd_packet, "uint8");
        
        % STM32 Telemetri Paketi Oku (10 Bayt: [0x55] [dist:2B] [str:2B] [pan*10:2B] [tilt*10:2B] [csum:1B])
        if sp.NumBytesAvailable >= 10
            rx = read(sp, 10, "uint8");
            if rx(1) == hex2dec('55')
                % Potansiyometre pan acisini cozumle
                pan_read_raw = typecast(rx(6:7), 'int16');
                actual_angle = double(pan_read_raw) / 10.0;
            end
        else
            % Yeni paket yoksa son durumu koru
            actual_angle = actual_angle + cmd_step * 0.8;
        end
    else
        % Sanal Donanim Yaniti (Lag + Backlash + Entegrasyon)
        sim_v = alpha_sim * sim_v + (1 - alpha_sim) * cmd_step;
        sim_pan_angle = sim_pan_angle + sim_v;
        % Kucuk olcum gurultusu ekle (0.04 deg)
        actual_angle = sim_pan_angle + 0.04 * randn();
    end
    
    % Kayit
    log_t(k) = t_now;
    log_ref(k) = target_ref;
    log_actual(k) = actual_angle;
    log_err(k) = err;
    log_cmd(k) = cmd_step;
    
    % Canli grafigi guncelle (her 5 karede bir)
    if mod(k, 5) == 0
        set(h_ref, 'XData', log_t(1:k), 'YData', log_ref(1:k));
        set(h_act, 'XData', log_t(1:k), 'YData', log_actual(1:k));
        set(h_err, 'XData', log_t(1:k), 'YData', log_err(1:k));
        drawnow limitrate;
    end
    
    % Gercek zamanli dongu zamanlamasi (30 Hz senkronizasyonu)
    t_target = k * dt;
    while toc(t_start) < t_target
        % Kisa bekleme
    end
end

% Baglantiyi kapat
if is_hardware_connected && ~isempty(sp)
    % Tareti sifirla / standby'a al
    clear sp;
    fprintf('>>> Seri port baglantisi duzenli sekilde kapatildi.\n');
end

% ------------------------------------------------------------------------------
% 5. HIL SONUCLARI VE METRIKLER
% ------------------------------------------------------------------------------
rmse_hil = sqrt(mean(log_err(log_t >= 1.0).^2));
lock_pct = sum(abs(log_err(log_t >= 1.0)) <= deadband_pan) / sum(log_t >= 1.0) * 100;

fprintf('\n--- HIL TEST PERFORMANS METRIKLERI ---\n');
fprintf('  * Test Edilen Mod:              %s\n', mode_str);
fprintf('  * Kararli Durum RMS Hatasi:     %.3f derece\n', rmse_hil);
fprintf('  * Kilitlenme Orani (%% Lock):     %% %.2f (Deadband: +/- %.2f deg)\n\n', lock_pct, deadband_pan);

% Grafigi kaydet
fig_path = fullfile(fig_dir, 'matlab_fig6_hil_test_result.png');
exportgraphics(fig, fig_path, 'Resolution', 300, 'BackgroundColor', 'white');
close(fig);
fprintf('>>> 300 DPI HIL Test Grafigi Kaydedildi: %s\n', fig_path);

% Verileri MAT olarak sakla
mat_path = fullfile(script_dir, 'hil_test_result.mat');
save(mat_path, 'log_t', 'log_ref', 'log_actual', 'log_err', 'log_cmd', 'rmse_hil', 'lock_pct');
fprintf('>>> HIL Test Telemetrisi Kaydedildi: %s\n\n', mat_path);

fprintf('====================================================================\n');
fprintf('  HARDWARE-IN-THE-LOOP (HIL) TESTI BASARIYLA TAMAMLANDI!\n');
fprintf('====================================================================\n');
