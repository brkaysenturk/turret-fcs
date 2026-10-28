"""
================================================================================
MODUL: AERODINAMIK HAVA DIRENCI VE RK4 BALISTIK HESAPLAYICI (BALLISTICS)
================================================================================
Bu modul; sunger dartlarin (micro foam dart) hafif kutle ve yuksek surukleme
katsayisi altindaki ucus mekanigini 4. Mertebe Runge-Kutta (RK4) sayisal
integratoru ile cozen, hava direncini (aerodynamic drag) hesaba katan balistik
motorudur.

Fiziksel Model:
  m * ax = -0.5 * rho * Cd * A * v * vx
  m * ay = -m * g - 0.5 * rho * Cd * A * v * vy
  Burada k = (0.5 * rho * Cd * A) / m (m^-1) surukleme balistik katsayisidir.

Temel Gorevler:
  1. compute_rk4_super_elevation: Hedef mesafesine (ve irtifasina) gore atis acisini
     (super-elevasyon) ve gercek ucus suresini RK4 shooting yontemiyle hesaplar.
  2. compute_rk4_flight_time: Kalman onleme (lead) hesabi icin hava direncli ucus suresini verir.
  3. compute_rk4_drop: Verilen mesafedeki yercekimi ve surukleme kaynakli dusus miktarini verir.
================================================================================
"""

import math
from typing import Tuple

from .config import CONFIG


def compute_drag_constant(
    mass_kg: float = None,
    cross_area_m2: float = None,
    drag_coeff: float = None,
    air_density: float = None,
) -> float:
    """Surukleme katsayisi k = (0.5 * rho * Cd * A) / m (birimi 1/metre)."""
    m_val = CONFIG.ballistics.projectile_mass_kg if mass_kg is None else mass_kg
    a_val = CONFIG.ballistics.projectile_cross_area_m2 if cross_area_m2 is None else cross_area_m2
    cd_val = CONFIG.ballistics.drag_coefficient if drag_coeff is None else drag_coeff
    rho_val = CONFIG.ballistics.air_density_kg_m3 if air_density is None else air_density
    m = max(1e-6, float(m_val))
    return float((0.5 * rho_val * cd_val * a_val) / m)


def solve_rk4_trajectory(
    dist_m: float,
    v0: float = 10.0,
    theta_rad: float = 0.0,
    g: float = 9.80665,
    k: float = 0.0346,
    target_y: float = 0.0,
    dt: float = 0.001,
    max_time_s: float = 1.5,
) -> Tuple[float, float]:
    """
    Verilen atis acisiyla (theta_rad) dartin hedef mesafesine (dist_m) ulasmasini
    RK4 integratoru ile simule eder.

    Donus:
      (t_flight, final_y_error)
      final_y_error: Hedef yatay mesafesine varildiginda dartin y koordinati ile target_y farki.
    """
    x = 0.0
    y = 0.0
    vx = v0 * math.cos(theta_rad)
    vy = v0 * math.sin(theta_rad)
    t = 0.0

    while x < dist_m and t < max_time_s:
        # Son adimda tam dist_m mesafesinde durmak icin dt kisitlamasi
        step_dt = dt
        if vx > 1e-4:
            rem_dist = dist_m - x
            if vx * step_dt > rem_dist:
                step_dt = max(1e-5, rem_dist / vx)

        def deriv(s_x, s_y, s_vx, s_vy):
            spd = math.hypot(s_vx, s_vy)
            a_x = -k * spd * s_vx
            a_y = -g - k * spd * s_vy
            return s_vx, s_vy, a_x, a_y

        k1_x, k1_y, k1_vx, k1_vy = deriv(x, y, vx, vy)

        k2_x, k2_y, k2_vx, k2_vy = deriv(
            x + 0.5 * step_dt * k1_x,
            y + 0.5 * step_dt * k1_y,
            vx + 0.5 * step_dt * k1_vx,
            vy + 0.5 * step_dt * k1_vy,
        )

        k3_x, k3_y, k3_vx, k3_vy = deriv(
            x + 0.5 * step_dt * k2_x,
            y + 0.5 * step_dt * k2_y,
            vx + 0.5 * step_dt * k2_vx,
            vy + 0.5 * step_dt * k2_vy,
        )

        k4_x, k4_y, k4_vx, k4_vy = deriv(
            x + step_dt * k3_x,
            y + step_dt * k3_y,
            vx + step_dt * k3_vx,
            vy + step_dt * k3_vy,
        )

        x += (step_dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x)
        y += (step_dt / 6.0) * (k1_y + 2.0 * k2_y + 2.0 * k3_y + k4_y)
        vx += (step_dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx)
        vy += (step_dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy)
        t += step_dt

    return t, (y - target_y)


def compute_rk4_super_elevation(
    dist_m: float,
    v0: float = 10.0,
    g: float = 9.80665,
    k: float = 0.0346,
    target_y: float = 0.0,
    max_iter: int = 5,
    tol_m: float = 0.001,
) -> Tuple[float, float]:
    """
    Hava direncli (drag) balistik ucus icin gereken super-elevasyon acisini (derece)
    ve ucus suresini (saniye) Newton/Secant shooting yontemiyle hesaplar.

    Donus:
      (super_elevation_deg, flight_time_s)
    """
    d = max(0.20, float(dist_m))
    v_sq = max(1e-3, v0 * v0)

    # Ilk tahmin: vakum (surtunmesiz) parabolik denkleminden
    sin_2theta = (g * d) / v_sq
    if sin_2theta >= 0.999:
        theta = math.radians(44.0)
    else:
        theta = 0.5 * math.asin(sin_2theta)

    if k <= 1e-6:
        # Surukleme kapali ise dogrudan vakum cozumu
        t_vac = d / (v0 * math.cos(theta))
        return float(math.degrees(theta)), float(t_vac)

    # Shooting (Atis) Dongusu: Secant / Newton adimi
    t_flight = d / v0
    for _ in range(max_iter):
        t_flight, err_y = solve_rk4_trajectory(
            dist_m=d, v0=v0, theta_rad=theta, g=g, k=k, target_y=target_y, dt=0.001
        )
        if abs(err_y) < tol_m:
            break
        # d_err/d_theta turevi yaklasik olarak d / cos(theta) mertebesindedir
        d_theta = err_y / max(0.5, d)
        theta -= d_theta
        # Mekanik limitler dahilinde kirp (-15 ile +35 derece araligi)
        theta = max(math.radians(-15.0), min(math.radians(35.0), theta))

    return float(math.degrees(theta)), float(t_flight)


def compute_rk4_drop(
    dist_m: float,
    v0: float = 10.0,
    g: float = 9.80665,
    k: float = 0.0346,
) -> Tuple[float, float]:
    """
    Namlunun hedefe yatay (0 derece) atildigi varsayimiyla gerceklesen
    dikey dusus miktarini (metre, pozitif asagi) ve ucus suresini (saniye) hesaplar.
    """
    d = max(0.20, float(dist_m))
    t_flight, final_y = solve_rk4_trajectory(
        dist_m=d, v0=v0, theta_rad=0.0, g=g, k=k, target_y=0.0, dt=0.001
    )
    drop_m = -final_y
    return float(drop_m), float(t_flight)
