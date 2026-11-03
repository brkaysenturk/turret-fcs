import os, subprocess, time, json, tempfile, csv
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD = ROOT / "webots" / "worlds" / "turret_fcs.wbt"

def run_webots_trial(label, profile=2, dob_enabled=False, dob_gain=0.70, sim_time_sec=14.0):
    tmp_dir = Path(tempfile.gettempdir()) / "webots_dob_eval"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    tuning_file = tmp_dir / f"tune_{label}.json"
    csv_out = tmp_dir / f"tlm_{label}.csv"
    if csv_out.exists():
        csv_out.unlink()

    payload = {
        "label": label,
        "dob_enabled": 1.0 if dob_enabled else 0.0,
        "dob_gain": float(dob_gain),
        "dob_tau_s": 0.15,
        "max_dob_speed_deg_s": 20.0,
    }
    tuning_file.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    env = os.environ.copy()
    env["PATH"] = r"C:\Program Files\Webots\msys64\mingw64\bin;" + env.get("PATH", "")
    env.update(
        PYTHONUTF8="1",
        WEBOTS_CONTROL_MODE="P",
        WEBOTS_BALLOON_PROFILE=str(profile),
        WEBOTS_TUNING_JSON=str(tuning_file),
        WEBOTS_TELEMETRY_PATH=str(csv_out),
        WEBOTS_HEADLESS="1",
        WEBOTS_MAX_SIM_TIME=str(sim_time_sec),
        WEBOTS_IGNORE_PROFILE_LIMITS="1",
    )

    print(f"[*] Calistiriliyor: {label} (Profil={profile}, DOB={dob_enabled}, Gain={dob_gain})...")
    proc = subprocess.Popen(
        [
            WEBOTS,
            "--port=1236",
            "--batch",
            "--minimize",
            "--mode=fast",
            "--stdout",
            "--stderr",
            str(WORLD),
        ],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        proc.wait(timeout=35)
    except subprocess.TimeoutExpired:
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True)

    time.sleep(2)
    if not csv_out.exists() or csv_out.stat().st_size < 1000:
        print(f"[!] HATA: Telemetri dosyasi olusmadi: {csv_out}")
        return None

    # Analiz
    with open(csv_out, newline="", encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r.get("target_detected") == "1"]

    if len(rows) < 50:
        print(f"[!] Yetersiz tespit: {len(rows)} kare")
        return None

    err_p = np.array([float(r["angular_err_pan"]) for r in rows])
    err_t = np.array([float(r["angular_err_tilt"]) for r in rows])
    cmd_p = np.array([float(r.get("cmd_pan_deg") or 0.0) for r in rows])
    is_locked = np.array([1 if r["is_locked"] == "1" else 0 for r in rows])

    diff_cmd = np.diff(cmd_p)
    moves = diff_cmd[np.abs(diff_cmd) > 1e-4]
    flip_rate = float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0) if len(moves) > 2 else 0.0

    return {
        "label": label,
        "profile": profile,
        "frames": len(rows),
        "rms_pan": float(np.sqrt(np.mean(err_p**2))),
        "mae_pan": float(np.mean(np.abs(err_p))),
        "max_pan": float(np.max(np.abs(err_p))),
        "rms_tilt": float(np.sqrt(np.mean(err_t**2))),
        "lock_pct": float(np.mean(is_locked) * 100.0),
        "flip_rate": flip_rate,
    }

if __name__ == "__main__":
    results = []

    # 1. Test: Profil 2 (Standart Sinus - Hareketli Hedef)
    print("\n" + "="*60)
    print("TEST 1: PROFIL 2 (STANDART SINUS - SUREKLI HAREKETLI HEDEF)")
    print("="*60)
    r_base_p2 = run_webots_trial("baseline_p2", profile=2, dob_enabled=False)
    r_dob_p2 = run_webots_trial("dob_p2", profile=2, dob_enabled=True, dob_gain=0.70)
    if r_base_p2: results.append(r_base_p2)
    if r_dob_p2: results.append(r_dob_p2)

    # 2. Test: Profil 5 (Zigzag - Sert Yon Degisimleri ve Overshoot Testi)
    print("\n" + "="*60)
    print("TEST 2: PROFIL 5 (ZIGZAG - SERT YON DEGISIMI VE OVERSHOOT TESTI)")
    print("="*60)
    r_base_p5 = run_webots_trial("baseline_p5", profile=5, dob_enabled=False)
    r_dob_p5 = run_webots_trial("dob_p5", profile=5, dob_enabled=True, dob_gain=0.70)
    if r_base_p5: results.append(r_base_p5)
    if r_dob_p5: results.append(r_dob_p5)

    print("\n" + "="*80)
    print(f"{'Kosu':<15} | {'Profil':<8} | {'Pan RMS':<10} | {'Pan MAE':<10} | {'Pan Max':<10} | {'Kilit %':<10} | {'Flip %':<8}")
    print("="*80)
    for r in results:
        print(f"{r['label']:<15} | {r['profile']:<8} | {r['rms_pan']:<10.3f} | {r['mae_pan']:<10.3f} | {r['max_pan']:<10.3f} | {r['lock_pct']:<10.1f} | {r['flip_rate']:<8.1f}")
    print("="*80)
