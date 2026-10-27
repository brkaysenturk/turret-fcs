%% ==============================================================================
% TURRET FCS // SIMULINK DIJITAL IKIZ MODEL INSA BETIGI (BUILD DIGITAL TWIN)
% ==============================================================================
% Bu betik; Turret FCS donaniminin (MG996R servo, STM32, kamera ve YOLO orneklemesi)
% tam bir Simulink dijital ikizini (turret_digital_twin.slx) programatik olarak
% insa eder ve kaydeder.
% 
% BLOK MIMARISI:
% 1) Referans Hedef Ureteci (Target Trajectory: Basamak, Sinus, Rampa)
% 2) FCS Kontrolcu (Tip-1 P Kontrolcu, Yumusak Olu Bolge, Hiz/Ivme Limiti)
% 3) Tasiyici Gecikme (Transport Delay: 80 ms kamera + haberlesme gecikmesi)
% 4) Mekanik Tesis (Backlash/Disli Boslugu, Servo Lag, Tip-1 Aci Entegratoru)
% 5) Optik Sensor (ZOH: 30 FPS YOLOv8 ornekleme gecikmesi ve kuantalama)
% ==============================================================================

clear; clc;

fprintf('====================================================================\n');
fprintf('  TURRET FCS // SIMULINK DIJITAL IKIZ MODELI INSA EDILIYOR          \n');
fprintf('====================================================================\n\n');

script_dir = fileparts(mfilename('fullpath'));
model_name = 'turret_digital_twin';
slx_path = fullfile(script_dir, [model_name, '.slx']);

% Model onceden aciksa kapat
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Uretim ve Gercek Telemetri Sistem Tanilama Parametreleri (turret_fcs/config.py SSOT)
% v2.1 Optuna TPE & Canli Telemetri sonuclari:
K_val = 0.985;              % Aktuator kazanci
tau_val = 0.0682;           % 68.2 ms servo zaman sabiti (CONFIG.mount.servo_tau)
theta_val = 0.0667;         % 66.7 ms tasiyici gecikme (2 tam kare @ 30 FPS)
backlash_val = 1.61;        % 1.61 deg MG996R mekanik disli boslugu (CONFIG.actuator.backlash_deg)
cmd_accel_val = 353.1;      % 353.1 deg/s^2 Pan ivme siniri (CONFIG.control.cmd_accel_pan_deg_s2)

Kp_val = 0.1953;            % Optuna TPE optimal kare basi kazanc (deg/kare)
deadband_val = 0.85;        % Pan yumusak olu bolgesi (deg)

% FCS Oransal Kazanci (Ayrik 30 FPS komutundan surekli zaman hizina donusum: Kp_cont = Kp_frame * 30)
fps_val = 30.0;
Kp_cont = Kp_val * fps_val; % 0.1953 * 30 = 5.859 s^-1

fprintf('>>> Model Parametreleri (Uretim CONFIG.py ile %100 Uyumlu):\n');
fprintf('  * Kp (Ayrik): %.4f deg/kare, Kp (Surekli): %.3f s^-1\n', Kp_val, Kp_cont);
fprintf('  * Olu Bolge: +/- %.2f deg, Servo Hiz Limiti: +/- 45 deg/s, Ivme Limiti: +/- %.1f deg/s^2\n', deadband_val, cmd_accel_val);
fprintf('  * Tau: %.1f ms, Olu Zaman (Delay): %.1f ms, Disli Boslugu: %.2f deg\n\n', ...
    tau_val * 1000, theta_val * 1000, backlash_val);

% Yeni Simulink Sistemi Olustur
new_system(model_name);
open_system(model_name);

% Cozucu Ayarlari
set_param(model_name, 'Solver', 'ode45', ...
                      'StopTime', '10.0', ...
                      'RelTol', '1e-4', ...
                      'AbsTol', '1e-5');

% Blok Konumlari ve Ekleme
% [Sol, Ust, Sag, Alt]

% 1. Hedef Referansi (Sinus Dalgasi: 0.15 Hz, 10 deg genlik)
add_block('simulink/Sources/Sine Wave', [model_name, '/Target_Angle'], ...
    'Position', [60, 140, 110, 180], ...
    'Amplitude', '10.0', ...
    'Frequency', '0.15 * 2 * pi', ...
    'Bias', '90.0');

% 2. Basamak Referansi (Alternatif test icin)
add_block('simulink/Sources/Step', [model_name, '/Step_Target'], ...
    'Position', [60, 220, 110, 260], ...
    'Time', '1.0', ...
    'Before', '90.0', ...
    'After', '95.0');

% 3. Referans Secici Manuel Anahtar
add_block('simulink/Signal Routing/Manual Switch', [model_name, '/Ref_Switch'], ...
    'Position', [150, 160, 190, 200]);

% 4. Hata Toplayici (Sum: Target - Feedback)
add_block('simulink/Math Operations/Sum', [model_name, '/Sum_Error'], ...
    'Inputs', '+-', ...
    'Position', [230, 165, 260, 195]);

% 5. Yumusak Olu Bolge (Dead Zone)
add_block('simulink/Discontinuities/Dead Zone', [model_name, '/Dead_Zone'], ...
    'Position', [290, 165, 335, 195], ...
    'LowerValue', sprintf('-%.4f', deadband_val), ...
    'UpperValue', sprintf('%.4f', deadband_val));

% 6. FCS Oransal Kazanc (Kp_cont = Kp * 30)
add_block('simulink/Math Operations/Gain', [model_name, '/Gain_Kp'], ...
    'Position', [365, 165, 410, 195], ...
    'Gain', sprintf('%.4f', Kp_cont));

% 7. Maksimum Servo Hizi Satürasyonu (+/- 45 deg/s)
add_block('simulink/Discontinuities/Saturation', [model_name, '/Velocity_Saturation'], ...
    'Position', [435, 165, 475, 195], ...
    'UpperLimit', '45.0', ...
    'LowerLimit', '-45.0');

% 8. Komut Ivme Sinirlayici (Rate Limiter: +/- 353.1 deg/s^2 Optuna TPE optimal)
add_block('simulink/Discontinuities/Rate Limiter', [model_name, '/Rate_Limiter'], ...
    'Position', [500, 165, 545, 195], ...
    'RisingSlewLimit', sprintf('%.1f', cmd_accel_val), ...
    'FallingSlewLimit', sprintf('-%.1f', cmd_accel_val));

% 9. Tasiyici Gecikme (Transport Delay: 50-80 ms kamera + UART)
add_block('simulink/Continuous/Transport Delay', [model_name, '/Transport_Delay'], ...
    'Position', [575, 165, 620, 195], ...
    'DelayTime', sprintf('%.4f', theta_val), ...
    'InitialOutput', '0.0');

% 10. Mekanik Disli Boslugu (Backlash Hysteresis)
add_block('simulink/Discontinuities/Backlash', [model_name, '/Backlash'], ...
    'Position', [650, 165, 695, 195], ...
    'BacklashWidth', sprintf('%.4f', backlash_val), ...
    'InitialOutput', '0.0');

% 11. Servo Dinamigi Lag (Transfer Fcn: K / (tau*s + 1))
add_block('simulink/Continuous/Transfer Fcn', [model_name, '/Servo_Dynamics'], ...
    'Position', [725, 160, 795, 200], ...
    'Numerator', sprintf('[%.4f]', K_val), ...
    'Denominator', sprintf('[%.4f 1]', tau_val));

% 12. Konum Entegratoru (Tip-1 Motor Entegrasyonu)
add_block('simulink/Continuous/Integrator', [model_name, '/Turret_Integrator'], ...
    'Position', [825, 165, 865, 195], ...
    'InitialCondition', '90.0', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', '180.0', ...
    'LowerSaturationLimit', '0.0');

% 13. Optik Ornekleme Sensoru (Zero-Order Hold: 30 FPS / 33.3 ms)
add_block('simulink/Discrete/Zero-Order Hold', [model_name, '/Camera_ZOH'], ...
    'Position', [650, 310, 695, 340], ...
    'SampleTime', '0.03333');

% 14. Scope (Gosterge)
add_block('simulink/Sinks/Scope', [model_name, '/Scope_Turret'], ...
    'Position', [935, 150, 975, 190], ...
    'NumInputPorts', '2');

% 15. To Workspace (Veri Kaydedici)
add_block('simulink/Sinks/To Workspace', [model_name, '/Out_Actual'], ...
    'Position', [935, 220, 995, 250], ...
    'VariableName', 'sim_actual_deg', ...
    'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [model_name, '/Out_Ref'], ...
    'Position', [250, 80, 310, 110], ...
    'VariableName', 'sim_ref_deg', ...
    'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [model_name, '/Out_Error'], ...
    'Position', [365, 80, 425, 110], ...
    'VariableName', 'sim_error_deg', ...
    'SaveFormat', 'Timeseries');

% ------------------------------------------------------------------------------
% BAGLANTILARI KUR (ADD_LINE)
% ------------------------------------------------------------------------------
add_line(model_name, 'Target_Angle/1', 'Ref_Switch/1');
add_line(model_name, 'Step_Target/1', 'Ref_Switch/2');
add_line(model_name, 'Ref_Switch/1', 'Sum_Error/1');
add_line(model_name, 'Ref_Switch/1', 'Out_Ref/1');

add_line(model_name, 'Sum_Error/1', 'Dead_Zone/1');
add_line(model_name, 'Sum_Error/1', 'Out_Error/1');

add_line(model_name, 'Dead_Zone/1', 'Gain_Kp/1');
add_line(model_name, 'Gain_Kp/1', 'Velocity_Saturation/1');
add_line(model_name, 'Velocity_Saturation/1', 'Rate_Limiter/1');
add_line(model_name, 'Rate_Limiter/1', 'Transport_Delay/1');
add_line(model_name, 'Transport_Delay/1', 'Backlash/1');
add_line(model_name, 'Backlash/1', 'Servo_Dynamics/1');
add_line(model_name, 'Servo_Dynamics/1', 'Turret_Integrator/1');

% Cikis baglantilari
add_line(model_name, 'Turret_Integrator/1', 'Scope_Turret/1');
add_line(model_name, 'Ref_Switch/1', 'Scope_Turret/2');
add_line(model_name, 'Turret_Integrator/1', 'Out_Actual/1');

% Geri Besleme (Feedback Loop)
add_line(model_name, 'Turret_Integrator/1', 'Camera_ZOH/1');
add_line(model_name, 'Camera_ZOH/1', 'Sum_Error/2');

% Modeli kaydet ve kapat
save_system(model_name, slx_path);
close_system(model_name);

fprintf('>>> Simulink Modeli Basariyla Olusturuldu ve Kaydedildi: %s\n\n', slx_path);
fprintf('====================================================================\n');
fprintf('  SIMULINK DIJITAL IKIZI HAZIR!\n');
fprintf('====================================================================\n');
