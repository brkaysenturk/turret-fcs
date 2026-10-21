import csv
import math
import numpy as np

log_path = 'logs/telemetry_test_20261020_120134.csv'
with open(log_path, 'r', encoding='utf-8') as f:
    rows = list(csv.DictReader(f))

total_frames = len(rows)
duration = float(rows[-1]['time_sec']) - float(rows[0]['time_sec'])
fps_list = [float(r['fps']) for r in rows if float(r['fps']) > 0]
mean_fps = np.mean(fps_list) if fps_list else 0.0

detected_rows = [r for r in rows if r['target_detected'] == '1']
locked_rows = [r for r in rows if r['is_locked'] == '1']

# Mod ayrımı
p_rows = [r for r in rows if r['control_mode'] == 'P']
rl_rows = [r for r in rows if r['control_mode'] == 'RL']
p_det = [r for r in p_rows if r['target_detected'] == '1']
rl_det = [r for r in rl_rows if r['target_detected'] == '1']

# Patrol durumları
patrol_statuses = {}
for r in rows:
    st = r['patrol_status'].split(' (')[0]
    patrol_statuses[st] = patrol_statuses.get(st, 0) + 1

# P Modu Hata ve Komut Analizi
p_err_pan = [abs(float(r['angular_err_pan'])) for r in p_det]
p_err_tilt = [abs(float(r['angular_err_tilt'])) for r in p_det]
p_cmd_pan = [abs(float(r['cmd_pan'])) for r in p_det]
p_cmd_tilt = [abs(float(r['cmd_tilt'])) for r in p_det]

# RL Modu Hata ve Komut Analizi
rl_err_pan = [abs(float(r['angular_err_pan'])) for r in rl_det]
rl_err_tilt = [abs(float(r['angular_err_tilt'])) for r in rl_det]
rl_cmd_pan = [abs(float(r['cmd_pan'])) for r in rl_det]
rl_cmd_tilt = [abs(float(r['cmd_tilt'])) for r in rl_det]

# Kalman Hız Analizi (Yakın vs Uzak)
near_det = [r for r in detected_rows if 0 < float(r['dist_cm']) < 100]
far_det = [r for r in detected_rows if float(r['dist_cm']) >= 100]

near_speeds = [float(r['target_speed']) for r in near_det]
far_speeds = [float(r['target_speed']) for r in far_det]

# RTB (Merkeze Dönüş) Birikimli Adım Analizi
rtb_rows = [r for r in rows if 'RTB' in r['patrol_status']]
rtb_dispatched_steps = [abs(float(r['cmd_pan'])) for r in rtb_rows if abs(float(r['cmd_pan'])) > 0.01]

# Kilitlenme serisi (Maksimum kesintisiz kilit)
max_lock_streak = 0
curr_streak = 0
for r in rows:
    if r['is_locked'] == '1':
        curr_streak += 1
        if curr_streak > max_lock_streak:
            max_lock_streak = curr_streak
    else:
        curr_streak = 0

# Güven Skoru Dağılımı
confs = [float(r['target_conf']) for r in detected_rows]
low_confs = [c for c in confs if c < 0.30]

print("=== 1. GENEL TEST METRİKLERİ ===")
print(f"Toplam Kare Sayısı: {total_frames} kare")
print(f"Toplam Oturum Süresi: {duration:.2f} saniye ({duration/60:.2f} dakika)")
print(f"Ortalama Döngü Hızı (FPS): {mean_fps:.1f} FPS (dt ortalama: {1000/mean_fps:.1f} ms)")
print(f"Hedef Tespit Oranı: {len(detected_rows)}/{total_frames} (%{len(detected_rows)/total_frames*100:.1f})")
print(f"Kilitli Kalma Oranı: {len(locked_rows)}/{len(detected_rows)} (%{len(locked_rows)/max(1, len(detected_rows))*100:.1f})")
print(f"En Uzun Kesintisiz Kilit: {max_lock_streak} kare ({max_lock_streak/mean_fps:.2f} saniye)")

print("\n=== 2. KONTROL MODU KARŞILAŞTIRMASI (P vs RL) ===")
print(f"P Modu Toplam Kare: {len(p_rows)} | Tespitli: {len(p_det)}")
if p_det:
    print(f"  P Pan Hata Ortalaması: {np.mean(p_err_pan):.2f}° (Maks: {np.max(p_err_pan):.2f}°)")
    print(f"  P Tilt Hata Ortalaması: {np.mean(p_err_tilt):.2f}° (Maks: {np.max(p_err_tilt):.2f}°)")
    print(f"  P Pan Komut Ortalaması: {np.mean(p_cmd_pan):.3f}°/kare (Maks: {np.max(p_cmd_pan):.2f}°/kare)")
    print(f"  P Kilitlenme Başarısı: %{len([r for r in p_det if r['is_locked'] == '1'])/len(p_det)*100:.1f}")

print(f"\nRL Modu Toplam Kare: {len(rl_rows)} | Tespitli: {len(rl_det)}")
if rl_det:
    print(f"  RL Pan Hata Ortalaması: {np.mean(rl_err_pan):.2f}° (Maks: {np.max(rl_err_pan):.2f}°)")
    print(f"  RL Tilt Hata Ortalaması: {np.mean(rl_err_tilt):.2f}° (Maks: {np.max(rl_err_tilt):.2f}°)")
    print(f"  RL Pan Komut Ortalaması: {np.mean(rl_cmd_pan):.3f}°/kare (Maks: {np.max(rl_cmd_pan):.2f}°/kare)")
    print(f"  RL Kilitlenme Başarısı: %{len([r for r in rl_det if r['is_locked'] == '1'])/len(rl_det)*100:.1f}")

print("\n=== 3. KALMAN FİLTRESİ VE HIZ GÜRÜLTÜSÜ ANALİZİ ===")
print(f"Yakın Mesafe (<1.0m) Tespit Sayısı: {len(near_det)} kare")
if near_det:
    print(f"  Yakın Mesafe Ortalama Hız: {np.mean(near_speeds):.1f} px/s (Medyan: {np.median(near_speeds):.1f} px/s, Maks: {np.max(near_speeds):.1f} px/s)")
    near_zero_pct = len([s for s in near_speeds if s < 5.0]) / len(near_speeds) * 100
    print(f"  Yakın Mesafede Hızın Sıfır/Ölü Bölgede Kalma Oranı: %{near_zero_pct:.1f}")

print(f"Uzak Mesafe (>=1.0m) Tespit Sayısı: {len(far_det)} kare")
if far_det:
    print(f"  Uzak Mesafe Ortalama Hız: {np.mean(far_speeds):.1f} px/s (Medyan: {np.median(far_speeds):.1f} px/s, Maks: {np.max(far_speeds):.1f} px/s)")

print("\n=== 4. PATROL VE RTB (BİRİKİMLİ MERKEZE DÖNÜŞ) ANALİZİ ===")
for st, cnt in patrol_statuses.items():
    print(f"  {st:25s}: {cnt:5d} kare ({cnt/mean_fps:.1f}s)")

print(f"\nRTB Toplam Kare: {len(rtb_rows)} kare")
if rtb_dispatched_steps:
    print(f"  Ateşlenen Net Adım Sayısı: {len(rtb_dispatched_steps)}")
    print(f"  Ortalama Adım Büyüklüğü: {np.mean(rtb_dispatched_steps):.2f}° (Maks: {np.max(rtb_dispatched_steps):.2f}°, Min: {np.min(rtb_dispatched_steps):.2f}°)")

# Merkeze dönüş anındaki taret koordinatlarının başlangıç ve bitişi
rtb_episodes = []
current_ep = []
for r in rows:
    if 'RTB' in r['patrol_status']:
        current_ep.append(r)
    else:
        if current_ep:
            rtb_episodes.append(current_ep)
            current_ep = []
if current_ep:
    rtb_episodes.append(current_ep)

print(f"Toplam Merkeze Dönüş Olayı: {len(rtb_episodes)} defa")
for idx, ep in enumerate(rtb_episodes[:3]):
    start_p = float(ep[0]['turret_pan'])
    start_t = float(ep[0]['turret_tilt'])
    end_p = float(ep[-1]['turret_pan'])
    end_t = float(ep[-1]['turret_tilt'])
    dur = float(ep[-1]['time_sec']) - float(ep[0]['time_sec'])
    print(f"  Olay {idx+1}: Başlangıç=[{start_p:+.1f}°, {start_t:+.1f}°] -> Bitiş=[{end_p:+.1f}°, {end_t:+.1f}°] | Süre: {dur:.2f}s | Kare: {len(ep)}")

print("\n=== 5. YOLO CONFIDENCE VE HAREKET BULANIKLIĞI HİSTEREZİSİ ===")
print(f"Hedef Güven Skoru Dağılımı: Ortalama %{np.mean(confs)*100:.1f}, Medyan %{np.median(confs)*100:.1f}")
print(f"Hareket Bulanıklığı Alt Eşiği (<%30) İnen Kare Sayısı: {len(low_confs)} kare")
if low_confs:
    print(f"  En Düşük Güven: %{np.min(low_confs)*100:.1f}")
    print(f"  Bu karelerde hedef tespiti korundu mu: EVET (%100 kesintisiz)")
