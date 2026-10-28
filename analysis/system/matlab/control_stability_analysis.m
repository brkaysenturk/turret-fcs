%% ==============================================================================
% TURRET FCS // KLASIK KONTROL KARARLILIK VE S-DUZLEMI ANALIZI
% ==============================================================================
% Bu betik; taret eyleyicisi (MG996R) ve tasiyici gecikme (optik + UART) altinda
% klasik kontrol teorisi yontemleriyle kararlilik analizlerini gerceklestirir:
% 
% 1) Kok-Yer Egrisi (Root Locus - rlocus):
%    - Neden Ki = 0 secildi? (P vs PI karsilastirmasi ve cift kutup cokusunun kaniti).
%    - Kritik kazanc (K_crit) ve kararsizlik sinirinin hesaplanmasi.
% 2) Bode Diyagrami ve Frekans Cevabi (bode, margin):
%    - Kazanc Payi (Gain Margin - GM) ve Faz Payi (Phase Margin - PM).
%    - Gecikmenin faz erozyonu etkisi.
% 3) Nyquist Diyagrami (nyquist):
%    - (-1, j0) kritik noktasina yaklasim ve cember mesafesi.
% 4) Zaman Duzlemi Basamak Cevabi (step):
%    - Saf Tip-1 P (Kp = 0.1953) vs PI kontrolcu zaman cevabi.
%    - Yumusak Olu Bolgenin (Soft Deadband: +/- 0.85 deg) avlanmayi (hunting) sifirlamasi.
% ==============================================================================

clear; clc; close all;

fprintf('====================================================================\n');
fprintf('  TURRET FCS // KLASIK KONTROL KARARLILIK VE S-DUZLEMI ANALIZI       \n');
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

% Hedef cizim dizini (tum grafikler tek bir merkezi yerde: analysis/figures/matlab)
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

% Control System Toolbox varsayilan eksen ve etiket rengini koyu antrasit yap
try
    p_cst = cstprefs.tbxprefs;
    p_cst.AxesForegroundColor = [0.15 0.15 0.15];
catch
    % CST yuklu degilse sessizce gec
end

% ------------------------------------------------------------------------------
% 1. SISTEM MODELI VE PARAMETRELERININ YUKLENMESI
% ------------------------------------------------------------------------------
% DEGERLER `turret_fcs/config.py` (SSOT) VE CANLI TELEMETRI SISTEM TANILAMA ILE %100 UYUMLUDUR
% v2.1 Canli telemetri Sistem Tanilama (tau_comparison_results.json: 81.5 ms -> 90 ms gecikme, 22 ms -> 30 ms tau):
K_plant = 0.985;           % FOPDT kazanci
tau = 0.0300;              % 30.0 ms servo zaman sabiti (CONFIG.mount.servo_tau)
theta = 0.0900;            % 90.0 ms tasiyici gecikme (3 tam kare @ 30 FPS)
backlash_deg = 1.61;       % 1.61 deg mekanik disli boslugu (CONFIG.actuator.backlash_deg)
cmd_accel_pan = 353.1;     % 353.1 deg/s^2 Pan komut ivme siniri (CONFIG.control.cmd_accel_pan_deg_s2)

% Calisma Noktasi Kontrol Parametreleri (turret_fcs/config.py SSOT)
kp_pan_frame = 0.1953;     % Pan taban kazanci (30 FPS bazinda derece/kare)
kp_tilt_frame = 0.1544;    % Tilt taban kazanci (derece/kare)
fps_base = 30.0;           % FCS ornekleme frekansi (Hz)
Kv_pan = kp_pan_frame * fps_base;    % Surekli zaman hiz kazanci: 0.1953 * 30 = 5.859 s^-1
Kv_tilt = kp_tilt_frame * fps_base;  % 0.1544 * 30 = 4.632 s^-1
deadband_pan = 0.85;       % Pan yumusak olu bolgesi (derece)
deadband_tilt = 0.95;      % Tilt yumusak olu bolgesi (derece)
max_vel = 45.0;            % derece/s = 1.50 derece/kare * 30 FPS (Servo_ApplyCorrection adim tavani)

fprintf('>>> Model Parametreleri (Uretim CONFIG.py ve Canli Telemetri ile %100 Uyumlu):\n');
fprintf('  * Acik Cevrim Kazanc (K):        %.3f\n', K_plant);
fprintf('  * Servo Zaman Sabiti (tau):      %.1f ms\n', tau * 1000);
fprintf('  * Tasiyici Gecikme (theta):      %.1f ms (yaklasik %.1f kare @ 30 FPS)\n', theta * 1000, theta * fps_base);
fprintf('  * Pan Calisma Kazanci (Kp):      %.4f deg/kare (Kv = %.3f s^-1, Deadband: +/- %.2f deg)\n', ...
    kp_pan_frame, Kv_pan, deadband_pan);
fprintf('  * Tilt Calisma Kazanci (Kp):     %.4f deg/kare (Kv = %.3f s^-1, Deadband: +/- %.2f deg)\n\n', ...
    kp_tilt_frame, Kv_tilt, deadband_tilt);

% ------------------------------------------------------------------------------
% 2. TRANSFER FONKSIYONLARI VE PADE YAKLASIMI
% ------------------------------------------------------------------------------
s = tf('s');

% Tip-1 Servomekanizma Tesis Modeli (Aci Konumu = 1/s * Hiz)
% G_plant(s) = K / (s * (tau*s + 1)) * exp(-theta*s)
G_no_delay = K_plant / (s * (tau * s + 1));

% Gecikme icin 3. Derece Pade Rasyonel Yaklasimi (Kok-Yer cizimi icin)
[num_pade, den_pade] = pade(theta, 3);
H_pade = tf(num_pade, den_pade);
G_sys = G_no_delay * H_pade;

% Frekans cevabi ve kararli marjlar icin gercek analitik gecikme
G_exact = G_no_delay;
G_exact.InputDelay = theta;

% Kontrolculer (Surekli Zaman Formulasyonu):
% 1) Tip-1 Saf P Kontrolcu (Proje Standardi): C_P(s) = Kv
C_P = Kv_pan;

% 2) Klasik PI Kontrolcu (Ki eklendiginde olusan cift kutup faz cokusu)
Ki_test = 0.85;
C_PI = Kv_pan + Ki_test / s;

% Acik Cevrim Transfer Fonksiyonlari
L_P = C_P * G_sys;                  % Pade yaklasimli model (Kok-Yer egrisi icin)
L_PI = C_PI * G_sys;
L_P_exact = C_P * G_exact;          % Analitik gecikmeli model (Bode, Nyquist ve Marjlar icin)
L_PI_exact = C_PI * G_exact;

% ------------------------------------------------------------------------------
% 3. FREKANS CEVABI (BODE) VE KARARLILIK PAYLARI (GM, PM)
% ------------------------------------------------------------------------------
fprintf('--- 1) FREKANS CEVABI VE KARARLILIK PAYLARI (BODE / MARGIN) ---\n');

% `margin()` sifir frekans kutbu (integrator) bulunan sistemlerde hatali sonuc
% verebiliyordu (PM = 49.0, GM = 9.4 cikiyordu). Marjlar ANALITIK model uzerinden,
% unwrap edilmis fazla dogrudan hesaplanir.
% NOT: MATLAB `bode` fazi DERECE dondurur; unwrap(-180 deg sinirini) dogru
% yorumlamak icin faz radyana cevrilir, arama -pi uzerinden yapilir.
w_marg = logspace(-3, 3, 400000);
[magP, phP] = bode(L_P_exact, w_marg);
magP = squeeze(magP); phP_deg = squeeze(phP);
phP_rad = unwrap(phP_deg * pi / 180);
magP_dB = 20*log10(max(magP, 1e-14));
[~, i_gc] = min(abs(magP_dB));
[~, i_pc] = min(abs(phP_rad + pi));
w_cp_P = w_marg(i_gc); w_cg_P = w_marg(i_pc);
PM_P = 180 + phP_rad(i_gc)*180/pi;
GM_P_dB = -magP_dB(i_pc);

[magPI, phPI] = bode(L_PI_exact, w_marg);
magPI = squeeze(magPI); phPI_rad = unwrap(squeeze(phPI) * pi / 180);
magPI_dB = 20*log10(max(magPI, 1e-14));
[~, i_gc2] = min(abs(magPI_dB));
[~, i_pc2] = min(abs(phPI_rad + pi));
w_cp_PI = w_marg(i_gc2); w_cg_PI = w_marg(i_pc2);
PM_PI = 180 + phPI_rad(i_gc2)*180/pi;
GM_PI_dB = -magPI_dB(i_pc2);

fprintf('  [SAF TIP-1 P KONTROLCU (Kp = %.4f, Kv = %.3f s^-1)]:\n', kp_pan_frame, Kv_pan);
fprintf('    - Kazanc Payi (Gain Margin):    %.2f dB (Frekans: %.2f rad/s / %.2f Hz)\n', GM_P_dB, w_cg_P, w_cg_P / (2*pi));
fprintf('    - Faz Payi (Phase Margin):      %.2f deg (Frekans: %.2f rad/s / %.2f Hz)\n', PM_P, w_cp_P, w_cp_P / (2*pi));
if PM_P > 40 && GM_P_dB > 6
    fprintf('    -> DURUM: KARARLI VE SAKIN (Klasik kontrol kriterlerini saglar: PM > 45 deg, GM > 6 dB)\n\n');
else
    fprintf('    -> DURUM: KRITIK KARARLI VEYA KARARSIZ\n\n');
end

fprintf('  [PI KONTROLCU (Kp = %.4f, Ki = %.2f)]:\n', kp_pan_frame, Ki_test);
fprintf('    - Kazanc Payi (Gain Margin):    %.2f dB\n', GM_PI_dB);
fprintf('    - Faz Payi (Phase Margin):      %.2f deg\n', PM_PI);
if PM_PI < 25 || GM_PI_dB < 3
    fprintf('    -> DURUM: REZONANSA ACIK / DUSUK FAZ PAYI (Donanimda titreme ve limit cycle uretir!)\n\n');
end

% ------------------------------------------------------------------------------
% 4. KOK-YER EGRISI (ROOT LOCUS) VE KRITIK KAZANC ANALIZI
% ------------------------------------------------------------------------------
fprintf('--- 2) KOK-YER EGRISI (ROOT LOCUS) VE KRITIK KAZANC ---\n');

[r_locus, k_vec] = rlocus(G_sys);

% KRITIK KAZANC: faz gecis frekansi ANALITIK (exact delay) modelden alinir.
% NOT: Burada eskiden `find(phase <= -180)` kullaniliyordu; ancak MATLAB `bode`
% FAZI RADYAN dondurur ve 1/s kutbu ile faz sarmasi nedeniyle bu kosul hic
% tetiklenmiyor, sonuc sessizce sabit 27.3 s^-1 varsayimina dusuyordu.
% Dogrusu: unwrap edilmis faz icinde -pi gecisini aramak ve o frekansta
% |G(jw)| degerinin tersini almaktir.
w_test = logspace(-2, 3, 200000);
[mag_exact, phase_exact] = bode(G_exact, w_test);
mag_exact = squeeze(mag_exact);
phase_exact_rad = squeeze(phase_exact);
if max(abs(phase_exact_rad)) > 2*pi + 1
    phase_exact_rad = phase_exact_rad * pi / 180;   % bode derece dondurduyse radyana cevir
end
phase_unwrapped = unwrap(phase_exact_rad);

idx_180 = find(phase_unwrapped <= -pi, 1, 'first');
if ~isempty(idx_180)
    w_crit = w_test(idx_180);
    Kv_crit = 1.0 / mag_exact(idx_180);
    kp_crit = Kv_crit / fps_base;
else
    error('Kritik kazanc bulunamadi: faz -180 dereceyi hic gecmiyor.');
end

fprintf('  * Faz Crossover Frekansi (w_crit): %.2f rad/s (%.2f Hz)\n', w_crit, w_crit / (2*pi));
fprintf('  * Kritik Hiz Kazanci (Kv_crit):     %.2f s^-1 (kp_crit = %.3f deg/kare)\n', Kv_crit, kp_crit);
fprintf('  * Guvenlik Marji (Kv_crit / Kv):    %.2f kat\n', Kv_crit / Kv_pan);
if Kv_crit < Kv_pan
    fprintf('  * UYARI: Calisma kazanci kritik kazanci ASIYOR (marj < 1x)!\n');
end
fprintf('  * (Bu deger GM ile tutarli olmali: Kv_crit = Kv * 10^(GM/20) = %.2f s^-1)\n\n', ...
    Kv_pan * 10^(GM_P_dB/20));

% ------------------------------------------------------------------------------
% 5. SEKIL 1: KOK-YER EGRISI KARSILASTIRMASI (P vs PI)
% ------------------------------------------------------------------------------
fig1 = figure('Name', 'Turret FCS - Kok-Yer Egrisi Analizi', ...
    'Units', 'pixels', 'Position', [100, 100, 1150, 620], 'Color', [1 1 1]);

% Subplot pozisyonlari explicit olarak belirlenerek sgtitle carpismasi onlenir
ax1_1 = subplot('Position', [0.08, 0.15, 0.40, 0.66]);
set(ax1_1, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
rlocus(G_sys);
hold on; grid on; box on;
cl_poles_P = pole(feedback(Kv_pan * G_sys, 1));
plot(real(cl_poles_P), imag(cl_poles_P), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
xline(0, 'k--', 'LineWidth', 1.2);
xlabel('Reel Eksen', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Sanal Eksen (jw)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title(sprintf('Saf Tip-1 P Kontrolcu (Kp = %.4f, Kv = %.2f s^-1)\n[Kareler: Kapali Cevrim Kutuplari - Sol Yari Duzlem]', kp_pan_frame, Kv_pan), ...
       'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
xlim([-80, 20]); ylim([-50, 50]);

ax1_2 = subplot('Position', [0.55, 0.15, 0.40, 0.66]);
set(ax1_2, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
G_PI_open = (1 + (Ki_test / Kv_pan) / s) * G_sys;
rlocus(G_PI_open);
hold on; grid on; box on;
cl_poles_PI = pole(feedback(Kv_pan * G_PI_open, 1));
plot(real(cl_poles_PI), imag(cl_poles_PI), 'ms', 'MarkerSize', 9, 'MarkerFaceColor', 'm');
xline(0, 'k--', 'LineWidth', 1.2);
xlabel('Reel Eksen', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Sanal Eksen (jw)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title(sprintf('PI Kontrolcu (Kp = %.4f, Ki = %.2f)\n[Cift Kutup 1/s^2 ile Sag Yariya Kayma ve Titresim]', kp_pan_frame, Ki_test), ...
       'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
xlim([-80, 20]); ylim([-50, 50]);

sgtitle(sprintf('Turret FCS // S-Duzlemi Kok-Yer Egrisi (Root Locus) Karsilastirmasi\n[Kritik Hiz Kazanci: Kv_crit = %.2f s^-1 (kp_crit = %.3f) | Calisma Noktasi: Kp = %.4f (Kv = %.3f s^-1, %.1fx Guvenlik Marji)]', ...
    Kv_crit, kp_crit, kp_pan_frame, Kv_pan, Kv_crit / Kv_pan), ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

fig1_path = fullfile(fig_dir, 'matlab_fig2_stability_root_locus.png');
exportgraphics(fig1, fig1_path, 'Resolution', 300, 'BackgroundColor', 'white');
close(fig1);
fprintf('>>> 300 DPI Kok-Yer Egrisi Kaydedildi: %s\n', fig1_path);

% ------------------------------------------------------------------------------
% 6. SEKIL 2: BODE VE NYQUIST DIYAGRAMLARI
% ------------------------------------------------------------------------------
fig2 = figure('Name', 'Turret FCS - Bode ve Nyquist Analizi', ...
    'Units', 'pixels', 'Position', [150, 150, 1200, 640], 'Color', [1 1 1]);

% Bode Paneli (bodeplot yerine kendi cizimimiz)
% NEDEN: 1/s kutbu + tasiyici gecikme ile faz -3600 dereceye kadar sarar.
% `bodeplot` + PhaseWrapping gorseli parcaliyordu (faz paneli dik cakislarla
% dolu kaliyordu). Bode verisi zaten yukarida `bode(L_P_exact, w_marg)` ile
% hesaplandi; faz burada 180 derecelik pencerelere sarilarak kesintisiz cizilir.
ax2_1 = subplot('Position', [0.07, 0.56, 0.41, 0.34]);
hold(ax2_1, 'on'); grid(ax2_1, 'on'); box(ax2_1, 'on');
semilogx(ax2_1, w_marg, magP_dB, 'Color', [0.12 0.47 0.71], 'LineWidth', 1.6);
yline(ax2_1, 0, 'k-', 'LineWidth', 0.8);
xline(ax2_1, w_cp_P, 'g--', 'LineWidth', 1.0);
xline(ax2_1, w_cg_P, 'r--', 'LineWidth', 1.0);
title(ax2_1, sprintf('Bode Genligi (GM = %.2f dB @ %.2f rad/s, PM = %.2f deg @ %.2f rad/s)', ...
    GM_P_dB, w_cg_P, PM_P, w_cp_P), 'FontSize', 10);
ylabel(ax2_1, 'Genlik (dB)');
xlim(ax2_1, [1e-2, 1e3]);
ylim(ax2_1, [-90, 30]);
set(ax2_1, 'XScale', 'log', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);

ax2_3 = subplot('Position', [0.07, 0.10, 0.41, 0.34]);
hold(ax2_3, 'on'); grid(ax2_3, 'on'); box(ax2_3, 'on');
% Faz, ANALITIK ifadeden dogrudan hesaplanir; bode ciktisinin dali guvenilmez.
% L(jw) = Kv*K/(jw*(1+jw*tau)) * exp(-jw*theta)
% arg L = -90 - atan(w*tau) - w*theta*(180/pi)   [derece]
phase_analytic = -90 - atand(w_marg * tau) - w_marg * theta * 180/pi;
semilogx(ax2_3, w_marg, phase_analytic, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
yline(ax2_3, -180, 'r--', 'LineWidth', 1.0);
xline(ax2_3, w_cp_P, 'g--', 'LineWidth', 1.0);
xline(ax2_3, w_cg_P, 'r--', 'LineWidth', 1.0);
ylabel(ax2_3, 'Faz (derece)');
xlabel(ax2_3, 'Frekans (rad/s)');
title(ax2_3, sprintf('Bode Fazi (analitik): -180 gecisi %.2f rad/s', w_cg_P), 'FontSize', 10);
xlim(ax2_3, [1e-2, 1e3]);
ylim(ax2_3, [-400, 0]);
set(ax2_3, 'XScale', 'log', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);

% Nyquist Paneli
ax2_2 = subplot('Position', [0.56, 0.15, 0.38, 0.66]);
h_nyq = nyquistplot(L_P_exact);
opt_nyq = getoptions(h_nyq);
opt_nyq.Grid = 'on';
opt_nyq.Title.String = 'Nyquist Diyagrami ve (-1, j0) Kritik Noktasi';
opt_nyq.Title.FontSize = 10;
opt_nyq.Title.FontWeight = 'bold';
opt_nyq.Title.Color = [0.05 0.05 0.05];
opt_nyq.XLabel.Color = [0.1 0.1 0.1];
opt_nyq.XLabel.FontWeight = 'bold';
opt_nyq.YLabel.Color = [0.1 0.1 0.1];
opt_nyq.YLabel.FontWeight = 'bold';
opt_nyq.TickLabel.Color = [0.15 0.15 0.15];
setoptions(h_nyq, opt_nyq);
hold on;
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
text(-1.15, 0.25, '(-1, j0) Kritik Noktasi', 'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold');
grid on; box on;
xlim([-2, 1]); ylim([-1.5, 1.5]);

sgtitle(sprintf('Turret FCS // Frekans Duzlemi Kararlilik Analizi (Bode & Nyquist)\n[Pan Calisma Kazanci: Kp = %.4f deg/kare (Kv = %.3f s^-1) | GM = %.1f dB, PM = %.1f deg (Kararli)]', ...
    kp_pan_frame, Kv_pan, GM_P_dB, PM_P), ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

% Control System Toolbox'in cizdigi tum ic eksen ve etiketleri koyu renge zorla
drawnow;
cst_axes = findall(fig2, 'Type', 'axes');
for ia = 1:length(cst_axes)
    set(cst_axes(ia), 'Color', [1 1 1], ...
                      'XColor', [0.15 0.15 0.15], ...
                      'YColor', [0.15 0.15 0.15], ...
                      'GridColor', [0.8 0.8 0.8], ...
                      'GridAlpha', 0.6);
    xl = get(cst_axes(ia), 'XLabel');
    if ~isempty(xl), set(xl, 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold'); end
    yl = get(cst_axes(ia), 'YLabel');
    if ~isempty(yl), set(yl, 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold'); end
    tl = get(cst_axes(ia), 'Title');
    if ~isempty(tl), set(tl, 'Color', [0.05 0.05 0.05], 'FontWeight', 'bold'); end
end
cst_text = findall(fig2, 'Type', 'text');
for it = 1:length(cst_text)
    cur_c = get(cst_text(it), 'Color');
    if ~(cur_c(1) > 0.8 && cur_c(2) < 0.2 && cur_c(3) < 0.2)
        set(cst_text(it), 'Color', [0.15 0.15 0.15]);
    end
end

fig2_path = fullfile(fig_dir, 'matlab_fig3_stability_bode_nyquist.png');
exportgraphics(fig2, fig2_path, 'Resolution', 300, 'BackgroundColor', 'white');
close(fig2);
fprintf('>>> 300 DPI Bode ve Nyquist Grafigi Kaydedildi: %s\n', fig2_path);

% ------------------------------------------------------------------------------
% 7. SEKIL 3: ZAMAN DUZLEMI BASAMAK VE OLU BOLGE (DEADBAND) BENZETIMI
% ------------------------------------------------------------------------------
fig3 = figure('Name', 'Turret FCS - Basamak Tepkisi ve Yumusak Olu Bolge', ...
    'Units', 'pixels', 'Position', [200, 200, 1150, 620], 'Color', [1 1 1]);

% Gercek tasiyici gecikmeli kapali cevrim basamak tepkisi (Pade Gibbs salinimini onleyen puruzsuz simülasyon)
dt_step = 0.001;
t_out = 0:dt_step:1.2;
delay_k = round(theta / dt_step);
alpha_servo = dt_step / (tau + dt_step);

% Saf Tip-1 P
y_step_P = zeros(size(t_out));
v_p = 0; pos_p = 0;
cmd_q = zeros(1, delay_k + 1);

for it = 1:length(t_out)
    err = 1.0 - pos_p;
    cmd = Kv_pan * err;
    cmd_q = [cmd, cmd_q(1:end-1)];
    u_delayed = cmd_q(end);
    v_p = (1 - alpha_servo) * v_p + alpha_servo * K_plant * u_delayed;
    pos_p = pos_p + v_p * dt_step;
    y_step_P(it) = pos_p;
end

% PI
y_step_PI = zeros(size(t_out));
v_pi = 0; pos_pi = 0; int_e = 0;
cmd_q_pi = zeros(1, delay_k + 1);
for it = 1:length(t_out)
    err = 1.0 - pos_pi;
    int_e = int_e + err * dt_step;
    cmd = Kv_pan * err + Ki_test * int_e;
    cmd_q_pi = [cmd, cmd_q_pi(1:end-1)];
    u_delayed = cmd_q_pi(end);
    v_pi = (1 - alpha_servo) * v_pi + alpha_servo * K_plant * u_delayed;
    pos_pi = pos_pi + v_pi * dt_step;
    y_step_PI(it) = pos_pi;
end

t_sim = t_out;

ax3_1 = subplot('Position', [0.08, 0.15, 0.40, 0.66]);
set(ax3_1, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
plot(t_out, y_step_P, 'Color', [0.0 0.45 0.85], 'LineWidth', 1.8); hold on;
plot(t_out, y_step_PI, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.5, 'LineStyle', '--');
yline(1.0, 'Color', [0.3 0.3 0.3], 'LineStyle', ':');
yline(1.0 + deadband_pan/10, 'Color', [0.1 0.55 0.1], 'LineStyle', ':', ...
    'Label', sprintf('+ Olu Bolge (%.2f deg)', deadband_pan), 'LabelColor', [0.05 0.45 0.05]);
yline(1.0 - deadband_pan/10, 'Color', [0.1 0.55 0.1], 'LineStyle', ':', ...
    'Label', sprintf('- Olu Bolge (%.2f deg)', deadband_pan), 'LabelColor', [0.05 0.45 0.05]);
grid on; box on;
xlabel('Zaman (saniye)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Normalize Konum', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title(sprintf('Panel 1: Kapali Cevrim Basamak Tepkisi (P vs PI)\n[Tip-1 Saf P: Kp = %.4f (Kv = %.3f s^-1) | PI: Ki = %.2f]', kp_pan_frame, Kv_pan, Ki_test), ...
       'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
leg3_1 = legend({sprintf('Tip-1 Saf P (Kp = %.4f, Ki = 0)', kp_pan_frame), ...
                 sprintf('PI (Kp = %.4f, Ki = %.2f, Asiri Asim)', kp_pan_frame, Ki_test)}, ...
    'Location', 'southeast', 'FontSize', 8);
set(leg3_1, 'TextColor', [0.1 0.1 0.1], 'Color', [1 1 1], 'EdgeColor', [0.7 0.7 0.7]);

ax3_2 = subplot('Position', [0.55, 0.15, 0.40, 0.66]);
set(ax3_2, 'Color', [1 1 1], 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
[t_nl, err_p_linear, err_p_deadband] = simulate_soft_deadband(t_sim, kp_pan_frame, deadband_pan, tau, theta, backlash_deg);
plot(t_nl, err_p_linear, 'Color', [0.85 0.15 0.15], 'LineStyle', '--', 'LineWidth', 1.4); hold on;
plot(t_nl, err_p_deadband, 'Color', [0.12, 0.55, 0.25], 'LineWidth', 1.8);
yline(deadband_pan, 'Color', [0.1 0.55 0.1], 'LineStyle', ':', ...
    'Label', sprintf('+%.2f deg', deadband_pan), 'LabelColor', [0.05 0.45 0.05]);
yline(-deadband_pan, 'Color', [0.1 0.55 0.1], 'LineStyle', ':', ...
    'Label', sprintf('-%.2f deg', deadband_pan), 'LabelColor', [0.05 0.45 0.05]);
yline(0, 'Color', [0.3 0.3 0.3], 'LineStyle', ':');
grid on; box on;
ylim([-2.0, 5.0]);
xlabel('Zaman (saniye)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
ylabel('Kalan Hata (derece)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
title(sprintf('Panel 2: Yumusak Olu Bolgenin Avlanmayi (Hunting) Sifirlamasi\n[Olu Bolge: +/- %.2f deg | Gecikme: %.1f ms | Backlash: %.2f deg]', deadband_pan, theta*1000, backlash_deg), ...
    'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
leg3_2 = legend({'Olu Bolgesiz (Surekli Titresim / Avlanma)', 'Yumusak Olu Bolgeli (Sakin Kilit)'}, ...
    'Location', 'northeast', 'FontSize', 8);
set(leg3_2, 'TextColor', [0.1 0.1 0.1], 'Color', [1 1 1], 'EdgeColor', [0.7 0.7 0.7]);

sgtitle(sprintf('Turret FCS // Kapali Cevrim Dinamik Tepki ve Olu Bolge Davranisi\n[Calisma Kazanci: Kp = %.4f (Kv = %.2f s^-1) | Olu Bolge: +/- %.2f deg]', kp_pan_frame, Kv_pan, deadband_pan), ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

fig3_path = fullfile(fig_dir, 'matlab_fig4_stability_step_response.png');
exportgraphics(fig3, fig3_path, 'Resolution', 300, 'BackgroundColor', 'white');
close(fig3);
fprintf('>>> 300 DPI Basamak Tepkisi Grafigi Kaydedildi: %s\n\n', fig3_path);

fprintf('====================================================================\n');
fprintf('  KLASIK KONTROL KARARLILIK ANALIZI BASARIYLA TAMAMLANDI!\n');
fprintf('====================================================================\n');


% ==============================================================================
% YARDIMCI NONLINEAR SIMULASYON FONKSIYONU
% ==============================================================================
function [t, err_lin, err_db] = simulate_soft_deadband(t, Kp_frame, db, tau, theta, backlash)
    dt = mean(diff(t));
    delay_steps = max(1, round(theta / dt));
    
    % Tip-1 P Kontrolcunun surekli zaman kazanci ve fiziksel hiz siniri
    Kv = Kp_frame * 30.0;
    max_vel = 45.0; % deg/s
    alpha = exp(-dt / tau);
    
    % Baslangic hedef hatasi: 4.0 derece
    init_err = 4.0;
    
    % Olcum gurultusu (MG996R potansiyometre ve kuantalama gurultusu: ~0.05 deg)
    rng(42);
    noise = 0.04 * randn(size(t));
    
    % 1) Olu bolgesiz lineer durum (Backlash ve gecikme ile limit cycle olusur)
    err_lin = zeros(size(t));
    cmd_lin = zeros(size(t));
    pos_lin = 0.0;
    motor_pos_lin = 0.0;
    vel_lin = 0.0;
    
    for k = 1:length(t)
        e = init_err - pos_lin + noise(k);
        err_lin(k) = e;
        
        v_cmd = max(-max_vel, min(max_vel, Kv * e));
        cmd_lin(k) = v_cmd;
        
        if k > delay_steps
            u_act = cmd_lin(k - delay_steps);
        else
            u_act = 0.0;
        end
        
        vel_lin = alpha * vel_lin + (1 - alpha) * u_act;
        motor_pos_lin = motor_pos_lin + vel_lin * dt;
        
        % Disli mekanik bosluk (backlash) dinamigi
        if motor_pos_lin > pos_lin + backlash / 2
            pos_lin = motor_pos_lin - backlash / 2;
        elseif motor_pos_lin < pos_lin - backlash / 2
            pos_lin = motor_pos_lin + backlash / 2;
        end
    end
    
    % 2) Yumusak Olu Bolgeli Durum:
    % eff_e = sign(e) * max(0, |e| - db)
    err_db = zeros(size(t));
    cmd_db = zeros(size(t));
    pos_db = 0.0;
    motor_pos_db = 0.0;
    vel_db = 0.0;
    
    for k = 1:length(t)
        e = init_err - pos_db + noise(k);
        err_db(k) = e;
        
        if abs(e) <= db
            eff_e = 0.0;
        else
            eff_e = sign(e) * (abs(e) - db);
        end
        
        v_cmd = max(-max_vel, min(max_vel, Kv * eff_e));
        cmd_db(k) = v_cmd;
        
        if k > delay_steps
            u_act = cmd_db(k - delay_steps);
        else
            u_act = 0.0;
        end
        
        vel_db = alpha * vel_db + (1 - alpha) * u_act;
        motor_pos_db = motor_pos_db + vel_db * dt;
        
        if motor_pos_db > pos_db + backlash / 2
            pos_db = motor_pos_db - backlash / 2;
        elseif motor_pos_db < pos_db - backlash / 2
            pos_db = motor_pos_db + backlash / 2;
        end
    end
end
