"""

================================================================================

BASKET CHALLENGE: STATIK KUPA ICIN OPTUNA TPE HASSAS TAKIP OPTIMIZASYONU

================================================================================

Tek ve saf senaryo (Statik Kupa):

- 3 Farkli Mesafe: 1.0m (Yakin), 2.0m (Nominal), 3.0m (Uzak)

- 3 Farkli Aci/Konum: Sol (X = -0.35m), Merkez (X = 0.0m), Sag (X = +0.35m)

- Toplam 9 kanonik kupa konumunda taretin kilitlenme hizi, kalici durum hassasiyeti,

  asimi ve motor titreşimi (hunting/chattering) optimize edilir.

- En iyi parametreler 'basket/models/cup_tracker_params.json' dosyasina kaydedilir.

================================================================================

"""



from __future__ import annotations



import json

import math

import sys

from pathlib import Path




import numpy as np

import optuna



optuna.logging.set_verbosity(optuna.logging.WARNING)



# Proje kokunu sys.path'e ekle

ROOT = Path(__file__).resolve().parents[2]

if str(ROOT) not in sys.path:

    sys.path.insert(0, str(ROOT))



from turret_fcs.config import CONFIG

from turret_fcs.rl_controller import RLController





class StaticCupSimulator:

    """MG996R motor dinamigi, disli boslugu ve kamera optigini iceren statik simulatör."""



    def __init__(self):

        self.cam_w = 640

        self.cam_h = 480

        self.cx0 = self.cam_w / 2.0

        self.cy0 = self.cam_h / 2.0

        self.fx = 705.436

        self.fy = 697.011



        self.tau = float(CONFIG.mount.servo_tau)          # 30 ms motor gecikmesi

        self.backlash_deg = float(CONFIG.actuator.backlash_deg) # 1.20 deg disli boslugu

        self.servo_deadband = float(CONFIG.actuator.servo_deadband_deg) # 0.10 deg kuantalama

        self.turret_z = 0.9817                            # Taret kamera yuksekligi (CAD v10: 0.71 + 0.1152 + 0.1879 - 0.0314)

        self.cup_base_z = 0.710                           # Masa yuzeyi yuksekligi

        self.cup_height = 0.097                           # Kupa yuksekligi (9.7 cm)



    def _apply_backlash(self, target_angle: float, last_output: float) -> float:

        delta = float(target_angle) - float(last_output)

        if delta > self.backlash_deg:

            return float(target_angle) - self.backlash_deg

        if delta < -self.backlash_deg:

            return float(target_angle) + self.backlash_deg

        return float(last_output)



    def run_scenario(

        self,

        cup_x: float,

        cup_y: float,

        tracker: RLController,

        aim_mouth_ratio: float = 0.35,

        sim_duration_s: float = 2.8,

        dt: float = 0.032

    ) -> dict:

        """Belirtilen kupa konumunda (X, Y) taretin kapal cevrim takip simlasyonunu kosturur."""

        tracker.reset()



        # Kupanin agiz merkezinin 3B koordinati (dartin girmesi gereken nokta)

        mouth_z = self.cup_base_z + self.cup_height * (1.0 - aim_mouth_ratio * 0.5)



        cmd_pan_deg = 0.0

        cmd_tilt_deg = 0.0

        act_pan_deg = 0.0

        act_tilt_deg = 0.0

        backlash_pan = 0.0

        backlash_tilt = 0.0



        # 3 Kare gecikme kuyrugu (kamera isleme boru hatti)

        delay_frames = 3

        history = []



        time_steps = int(sim_duration_s / dt)

        err_records = []

        cmd_pan_history = []

        cmd_tilt_history = []



        time_to_lock = None

        LOCK_THRESHOLD_DEG = 0.75  # 0.75 derece altina inince kilitlendi sayilir



        for step in range(time_steps):

            t = step * dt



            # Kupanin tarete gore bagil konumu (Taretin pan ve tilt acilarina gore projeksiyon)

            # Gercek dunyadaki hedef acilari:

            target_pan_deg = math.degrees(math.atan2(cup_x, cup_y))

            target_tilt_deg = math.degrees(math.atan2(mouth_z - self.turret_z, cup_y))



            # Kameranin mevcut bakis acisina gore acisal hata:

            err_pan = target_pan_deg - act_pan_deg

            err_tilt = target_tilt_deg - act_tilt_deg



            # Piksel koordinatlarina donusum (Gecikmeli algilama)

            px = self.cx0 + math.tan(math.radians(err_pan)) * self.fx

            py = self.cy0 - math.tan(math.radians(err_tilt)) * self.fy



            # Kutu yuksekligi (kamera projeksiyonu):





            history.append((px, py, cup_y))

            if len(history) > delay_frames:

                delayed_px, delayed_py, delayed_dist = history[-delay_frames]

            else:

                delayed_px, delayed_py, delayed_dist = px, py, cup_y



            # 1. RL Controller komut adimini hesapla

            d_p, d_t = tracker.predict_angles(

                bx=delayed_px,

                by=delayed_py,

                center_x=self.cx0,

                center_y=self.cy0,

                screen_w=self.cam_w,

                screen_h=self.cam_h,

                vx=0.0,

                vy=0.0,

                dist_m=delayed_dist,

                dt=dt

            )



            # 2. Komut biriktirici ve mekanik limitler

            cmd_pan_deg = float(np.clip(cmd_pan_deg + d_p, -75.0, 75.0))

            cmd_tilt_deg = float(np.clip(cmd_tilt_deg + d_t, -25.0, 25.0))

            cmd_pan_history.append(cmd_pan_deg)

            cmd_tilt_history.append(cmd_tilt_deg)



            # 3. 0.10 derece UART kuantalamasi

            q_pan = round(cmd_pan_deg / self.servo_deadband) * self.servo_deadband

            q_tilt = round(cmd_tilt_deg / self.servo_deadband) * self.servo_deadband



            # 4. 1. Derece Motor Gecikmesi (Tau = 30 ms)

            alpha_m = dt / (self.tau + dt)

            raw_act_pan = (1.0 - alpha_m) * act_pan_deg + alpha_m * q_pan

            raw_act_tilt = (1.0 - alpha_m) * act_tilt_deg + alpha_m * q_tilt



            # 5. Mekanik Disli Boslugu (Backlash)

            act_pan_deg = self._apply_backlash(raw_act_pan, backlash_pan)

            backlash_pan = act_pan_deg

            act_tilt_deg = self._apply_backlash(raw_act_tilt, backlash_tilt)

            backlash_tilt = act_tilt_deg



            # Metrik kaydi

            tot_err = math.hypot(err_pan, err_tilt)

            err_records.append(tot_err)



            if tot_err < LOCK_THRESHOLD_DEG and time_to_lock is None:

                time_to_lock = t



        # Metriklerin hesaplanmasi:

        # Son 1.0 saniyelik kalici durum (steady-state)

        ss_steps = int(1.0 / dt)

        ss_errs = err_records[-ss_steps:]

        steady_state_rms = float(np.sqrt(np.mean(np.square(ss_errs))))



        # Komut yon degistirme orani (Hunting / Chattering)

        pan_diffs = np.diff(cmd_pan_history[-ss_steps:])

        tilt_diffs = np.diff(cmd_tilt_history[-ss_steps:])

        pan_revs = np.sum((pan_diffs[:-1] * pan_diffs[1:]) < -1e-5)

        tilt_revs = np.sum((tilt_diffs[:-1] * tilt_diffs[1:]) < -1e-5)

        hunting_ratio = float((pan_revs + tilt_revs) / (2.0 * max(1, ss_steps)))



        settling = time_to_lock if time_to_lock is not None else sim_duration_s



        return {

            "settling_time_s": settling,

            "steady_state_rms_deg": steady_state_rms,

            "hunting_ratio": hunting_ratio,

            "final_err_deg": err_records[-1]

        }





# 9 Kanonik Test Senaryosu: 3 Mesafe x 3 Aci

TEST_SCENARIOS = [

    # (cup_x, cup_y, name, weight)

    (-0.35, 1.0, "1m_Sol", 1.2),

    (0.00, 1.0, "1m_Merkez", 1.3),

    (+0.35, 1.0, "1m_Sag", 1.2),



    (-0.35, 2.0, "2m_Sol", 1.0),

    (0.00, 2.0, "2m_Merkez", 1.1),

    (+0.35, 2.0, "2m_Sag", 1.0),



    (-0.35, 3.0, "3m_Sol", 0.9),

    (0.00, 3.0, "3m_Merkez", 1.0),

    (+0.35, 3.0, "3m_Sag", 0.9),

]





def objective(trial: optuna.Trial) -> float:

    """Optuna TPE amac fonksiyonu: 9 kanonik senaryoda agirlikli maliyeti minimize eder."""

    deadband_pan = trial.suggest_float("deadband_pan_deg", 0.15, 0.45)

    deadband_tilt = trial.suggest_float("deadband_tilt_deg", 0.15, 0.45)

    taper_pan = trial.suggest_float("taper_pan_deg", 0.10, 0.35)

    taper_tilt = trial.suggest_float("taper_tilt_deg", 0.10, 0.35)

    in_flight_damping = trial.suggest_float("in_flight_damping", 0.05, 0.40)

    reversal_damping = trial.suggest_float("reversal_damping", 0.30, 0.75)

    aim_mouth_ratio = trial.suggest_float("aim_mouth_ratio", 0.25, 0.45)



    # RL Controller'i bu hiperparametrelerle olustur

    controller = RLController(

        deadband_pan_deg=deadband_pan,

        deadband_tilt_deg=deadband_tilt,

        taper_pan_deg=taper_pan,

        taper_tilt_deg=taper_tilt,

        in_flight_damping=in_flight_damping,

        reversal_damping=reversal_damping,

        soft_deadband_enabled=True

    )



    sim = StaticCupSimulator()

    scenario_costs = []



    for cup_x, cup_y, _, weight in TEST_SCENARIOS:

        res = sim.run_scenario(

            cup_x=cup_x,

            cup_y=cup_y,

            tracker=controller,

            aim_mouth_ratio=aim_mouth_ratio

        )



        # Senaryo Maliyet Fonksiyonu:

        # - Hizli kilitlenme (< 1.2s hedef)

        # - Kucuk kalici durum hatasi (< 0.35 deg hedef)

        # - Sifir hunting/titreme

        cost = (

            2.0 * res["settling_time_s"]

            + 6.0 * res["steady_state_rms_deg"]

            + 15.0 * res["hunting_ratio"]

            + 4.0 * res["final_err_deg"]

        ) * weight



        scenario_costs.append(cost)



    # Robust Cost: Ortalama maliyet + 0.30 * En kotu senaryo maliyeti

    mean_cost = float(np.mean(scenario_costs))

    worst_cost = float(np.max(scenario_costs))

    total_cost = mean_cost + 0.30 * worst_cost



    return total_cost





def main(n_trials: int = 100):

    print("=" * 70)

    print("BASKET CHALLENGE: STATIK KUPA ICIN OPTUNA TPE OPTIMIZASYONU")

    print(f"Toplam Deneme Sayisi: {n_trials} | 9 Kanonik Senaryo (1.0m, 2.0m, 3.0m)")

    print("=" * 70)



    sampler = optuna.samplers.TPESampler(seed=42)

    study = optuna.create_study(direction="minimize", sampler=sampler)



    study.optimize(objective, n_trials=n_trials, show_progress_bar=False)



    print("\n" + "=" * 70)

    print("OPTIMIZASYON TAMAMLANDI!")

    print(f"En Iyi Robust Maliyet: {study.best_value:.4f}")

    print("=" * 70)

    print("BULUNAN EN IYI PARAMETRELER:")

    for k, v in study.best_params.items():

        print(f"  {k:22s}: {v:.4f}")



    # En iyi sonuclari kaydet

    out_dir = Path(__file__).resolve().parent / "models"

    out_dir.mkdir(parents=True, exist_ok=True)

    out_path = out_dir / "cup_tracker_params.json"



    with open(out_path, "w", encoding="utf-8") as f:

        json.dump(study.best_params, f, indent=4)



    print(f"\n[OK] En iyi parametreler kaydedildi: {out_path}")

    print("=" * 70)





if __name__ == "__main__":

    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 100

    main(trials)

