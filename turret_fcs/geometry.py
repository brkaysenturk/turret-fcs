"""Kamera goruntu koordinatlarinda (x saga, y asagi, z ileri) 3B pinhole isin geometrisi."""

import numpy as np


def rotated_pixel_shift(point, pan_deg, tilt_deg, fx, fy):
    """Kamera pan/tilt donusu altinda kenar perspektifini de iceren piksel kaymasi.

    point, optik merkeze (principal point) gore bagil piksel koordinatidir.
    Pozitif pan goruntuyu sola, pozitif tilt ise asagi kaydirir (once yaw, sonra pitch).
    """
    x, y = np.asarray(point, dtype=float) / (float(fx), float(fy))
    pan, tilt = np.radians([float(pan_deg), float(tilt_deg)])
    ray = np.array([
        np.cos(pan) * x - np.sin(pan),
        y,
        np.sin(pan) * x + np.cos(pan),
    ], dtype=float)
    ray = np.array([
        ray[0],
        np.cos(tilt) * ray[1] + np.sin(tilt) * ray[2],
        -np.sin(tilt) * ray[1] + np.cos(tilt) * ray[2],
    ], dtype=float)
    if ray[2] <= 1e-6:
        return np.zeros(2, dtype=float)
    return ray[:2] / ray[2] * (float(fx), float(fy)) - np.asarray(point, dtype=float)

