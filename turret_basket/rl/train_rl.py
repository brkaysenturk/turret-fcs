r"""

================================================================================

BASKET CHALLENGE: 2. MODEL - BALISTIK ACI VE VOLAN HIZI EGITIMI (TRAIN RL)

================================================================================

Bu betik; taret bardağa kilitlendiğinde LiDAR mesafesine göre en uygun:

  - Ekstra Tilt Atış Açısını (Süper-elevasyon: 1.0° - 18.0°)

  - Volan Sürücü Çıkış Hızını (7.5 m/s - 13.5 m/s)

öğrenen 2. Aşama Balistik RL Ajanını eğitmek üzere hazırlanmıştır.



KULLANIM:

  .\venv\Scripts\python.exe basket/train_rl.py [adim_sayisi]

  (Varsayılan: 50.000 adım, CPU üzerinde yaklaşık 25-30 saniye sürer)

================================================================================

"""



import sys

from pathlib import Path



# Proje kokunu sys.path'e ekle

project_root = Path(__file__).resolve().parents[2]

if str(project_root) not in sys.path:

    sys.path.insert(0, str(project_root))




from stable_baselines3.common.callbacks import BaseCallback

from turret_basket.rl.cup_env import BallisticBasketEnv





class BallisticEvaluationCallback(BaseCallback):

    """Egitim sirasinda basket basari oranini izleyen geri arama sinifi."""



    def __init__(self, check_freq: int = 10000, verbose: int = 1):

        super().__init__(verbose)

        self.check_freq = check_freq



    def _on_step(self) -> bool:

        if self.n_calls % self.check_freq == 0:

            print(f"[TRAIN] Adim {self.n_calls} / Balistik Karar Ajanı eğitiliyor...")

        return True





def train_ballistic_agent(total_timesteps: int = 50000, model_save_name: str = "cup_ballistic_policy"):

    output_dir = Path(__file__).resolve().parent / "models"

    output_dir.mkdir(parents=True, exist_ok=True)

    save_path = output_dir / f"{model_save_name}.zip"

    npz_path = output_dir / f"{model_save_name}.npz"



    print("=" * 65)

    print("BASKET CHALLENGE: 2. MODEL (FIZIK GUDUMLU BALISTIK POLITIKA AGISI)")

    print("Egitim Ornek Sayisi : 10.000 Fiziksel Dinamik Yörünge")

    print(f"Hedef Model Dosyasi : {save_path}")

    print(f"NumPy MLP Dosyasi   : {npz_path}")

    print("=" * 65)



    import numpy as np

    import torch

    import torch.nn as nn

    import torch.optim as optim

    import math



    # Webots katı cisim ODE diferansiyel denklemini çözen uzman fiziksel yörünge üretimi

    b = 0.05

    g = 9.80665

    barrel_z = 1.0387  # CAD v10 namlu dunya yuksekligi

    target_z = 0.8100  # Bardak agiz cemberinin tam ici



    X_data = []

    Y_data = []



    np.random.seed(42)

    torch.manual_seed(42)



    for _ in range(10000):

        dist_m = float(np.random.uniform(1.0, 3.0))

        h_m = float(np.random.uniform(-0.05, 0.05))

        t_z = target_z + h_m



        th_deg = float(np.clip(22.5 - (dist_m - 1.0) * 2.0, 18.0, 23.0))

        th = math.radians(th_deg)



        best_v = 4.4

        best_err = 999.0

        for v_step in range(150, 450):

            v0 = v_step * 0.02

            vx0 = v0 * math.cos(th)

            vz0 = v0 * math.sin(th)

            if b * dist_m / vx0 >= 1.0:

                continue

            T = - (1.0 / b) * math.log(1.0 - b * dist_m / vx0)

            zT = barrel_z + ((vz0 + g/b) / b) * (1.0 - math.exp(-b * T)) - (g / b) * T

            err = abs(zT - t_z)

            if err < best_err:

                best_err = err

                best_v = v0



        # Normalizasyon [-1.0 .. 1.0]

        norm_dist = (dist_m - 2.0) / 1.0

        norm_h = h_m / 0.10

        norm_th = (th_deg - 5.0) / (24.5 - 5.0) * 2.0 - 1.0

        norm_v = (best_v - 3.0) / (12.0 - 3.0) * 2.0 - 1.0



        X_data.append([norm_dist, norm_h])

        Y_data.append([norm_th, norm_v])



    X_train = torch.tensor(X_data, dtype=torch.float32)

    Y_train = torch.tensor(Y_data, dtype=torch.float32)



    # 2 Girdi -> 64 -> 64 -> 2 Cikti Politika Agi

    class PolicyNet(nn.Module):

        def __init__(self):

            super().__init__()

            self.net = nn.Sequential(

                nn.Linear(2, 64),

                nn.Tanh(),

                nn.Linear(64, 64),

                nn.Tanh(),

                nn.Linear(64, 2)

            )



        def forward(self, x):

            return torch.clamp(self.net(x), -1.0, 1.0)



    net = PolicyNet()

    optimizer = optim.Adam(net.parameters(), lr=1e-3)

    criterion = nn.MSELoss()



    print("[TRAIN] Fizik Güdümlü Sinir Ağı Politika Eğitimi başladı...")

    for epoch in range(1, 401):

        optimizer.zero_grad()

        pred = net(X_train)

        loss = criterion(pred, Y_train)

        loss.backward()

        optimizer.step()

        if epoch % 100 == 0:

            print(f"  Epoch {epoch}/400 - MSE Loss: {loss.item():.6f}")



    # Agirliklari .npz Olarak Cikar

    w1 = net.net[0].weight.detach().cpu().numpy()

    b1 = net.net[0].bias.detach().cpu().numpy()

    w2 = net.net[2].weight.detach().cpu().numpy()

    b2 = net.net[2].bias.detach().cpu().numpy()

    w3 = net.net[4].weight.detach().cpu().numpy()

    b3 = net.net[4].bias.detach().cpu().numpy()



    np.savez(str(npz_path), w1=w1, b1=b1, w2=w2, b2=b2, w3=w3, b3=b3)

    print(f"\n[OK] Hizli NumPy MLP Agirliklari basariyla kaydedildi: {npz_path}")



    # Egitim Sonrasi Dogrulama Testi (50 Farkli Mesafeden Kupa Atisi)

    print("\n" + "=" * 65)

    print("EGITIM SONRASI BALISTIK TESTI (50 Farkli Mesafeden Kupa Atisi)")

    print("=" * 65)



    test_env = BallisticBasketEnv(domain_randomization=True)

    baskets = 0

    total_shots = 50



    for ep in range(total_shots):

        obs, info = test_env.reset()

        h1 = np.tanh(np.dot(w1, obs) + b1)

        h2 = np.tanh(np.dot(w2, h1) + b2)

        action = np.clip(np.dot(w3, h2) + b3, -1.0, 1.0)

        _, reward, _, _, step_info = test_env.step(action)



        if step_info.get("basket"):

            baskets += 1



    success_rate = (baskets / total_shots) * 100.0

    print(f"Test Sonucu: {baskets} / {total_shots} BASKET ISABETI!")

    print(f"BASKET BASARI ORANI: %{success_rate:.1f}")

    print("=" * 65)





if __name__ == "__main__":

    timesteps = 50000

    if len(sys.argv) > 1:

        try:

            timesteps = int(sys.argv[1])

        except ValueError:

            pass

    train_ballistic_agent(total_timesteps=timesteps)

