import time
from pathlib import Path
import cv2
from stable_baselines3 import PPO
from turret_env import TurretEnv


def run_interactive_test():
    current_dir = Path(__file__).resolve().parent
    model_path = current_dir / "models" / "rl_v2-1.zip"

    if not model_path.exists():
        print(f"[HATA] Model bulunamadi: {model_path}")
        print("Lutfen once 'python train_rl.py' calistirarak modeli egitin.")
        return

    print("==================================================")
    print("  TURRET FCS // RL İNTERAKTİF TEST & SİMÜLASYON   ")
    print("==================================================")
    print(f"[Model Yükleniyor]: {model_path.name}")
    model = PPO.load(str(model_path))

    env = TurretEnv(render_mode="human")
    num_episodes = 10

    for episode in range(1, num_episodes + 1):
        obs, _ = env.reset()
        done = False
        total_reward = 0.0
        step_idx = 0

        print(f"\n>>> [BÖLÜM {episode}/{num_episodes}] Senaryo: {env.trajectory_type.upper()}")

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            step_idx += 1

            env.render()
            done = terminated or truncated

            # 'q' tusuna basilirsa cik
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[Kullanıcı İptali] Test durduruldu.")
                env.close()
                return

        print(f"    Sonuç: Adım: {step_idx} | Toplam Skor (Reward): {total_reward:.1f}")
        time.sleep(0.5)

    env.close()
    print("\n[OK] Tüm test bölümleri başarıyla tamamlandı.")


if __name__ == "__main__":
    run_interactive_test()
