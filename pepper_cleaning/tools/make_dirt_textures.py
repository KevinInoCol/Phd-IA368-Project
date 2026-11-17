"""Genera las texturas del piso sucio: una textura periódica de suciedad cortada en N×N baldosas.

    python3 tools/make_dirt_textures.py [--n 4] [--px 128] [--seed 7]

Ruido filtrado en frecuencia (periódico por construcción, vía FFT): manchas grandes + polvo fino.
La baldosa (i, j) del piso usa el trozo (i mod N, j mod N) del atlas (coordenadas UV de la malla de
dirt_floor), así que las juntas no se ven y el patrón se repite cada N baldosas.
Salida: models/dirt/materials/textures/dirt_atlas.png, opaca: la
suciedad (alfa = cantidad) ya mezclada con el gris del ground_plane del museo (difuso 0.8) en espacio
lineal y guardada en sRGB; la baldosa usa difuso 1, así que lo limpio coincide con el piso. Gazebo sólo hace
transparencia por umbral (alpha test) en las texturas, por eso no se usa el alfa. Al limpiar se borra la
baldosa y aparece el piso real.
"""

import argparse
import os

import numpy as np
from PIL import Image

FLOOR_LINEAR = 0.8   # difuso del ground_plane del museo (lineal)


def srgb_to_linear(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(c):
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * np.clip(c, 0, 1) ** (1 / 2.4) - 0.055)


def periodic_noise(size, scale, rng):
    """Ruido periódico con energía alrededor de la longitud de onda `scale` (en px)."""
    f = np.fft.fftfreq(size)
    fx, fy = np.meshgrid(f, f)
    r = np.hypot(fx, fy) * scale
    spectrum = np.exp(-r ** 2) * (r > 0)
    phase = np.exp(2j * np.pi * rng.random((size, size)))
    n = np.real(np.fft.ifft2(spectrum * phase))
    return (n - n.mean()) / (n.std() + 1e-9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--px", type=int, default=128)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    size = a.n * a.px
    blobs = periodic_noise(size, size / 6, rng)        # manchas grandes
    mid = periodic_noise(size, size / 18, rng)
    fine = periodic_noise(size, size / 120, rng)       # polvo
    m = 0.6 * blobs + 0.3 * mid + 0.1 * fine
    alpha = np.clip(0.5 + 0.3 * m, 0.08, 0.92)       # todo sucio, con zonas más cargadas
    spots = np.clip((0.7 * blobs + 0.5 * mid - 1.1) * 1.5, 0, 1)
    alpha = np.clip(alpha + 0.4 * spots, 0, 0.97)
    t = np.clip(0.5 + 0.25 * mid + 0.15 * fine, 0, 1)  # color: barro marrón ↔ polvo gris
    mud, dust = np.array([92, 68, 44]), np.array([120, 112, 100])
    rgb = (mud[None, None] * (1 - t[..., None]) + dust[None, None] * t[..., None])
    rgb *= (1 - 0.35 * spots)[..., None]
    # mezcla en espacio lineal: piso (FLOOR_LINEAR) ↔ suciedad, y salida en sRGB (Gazebo convierte
    # la textura sRGB → lineal y la multiplica por el difuso 1 del material de la baldosa)
    lin = FLOOR_LINEAR * (1 - alpha[..., None]) + srgb_to_linear(rgb / 255.0) * alpha[..., None]
    out_rgb = (linear_to_srgb(lin) * 255).round().astype(np.uint8)
    out = os.path.join(os.path.dirname(__file__), "..", "models", "dirt", "materials", "textures")
    os.makedirs(out, exist_ok=True)
    # un solo atlas: la baldosa (i, j) del piso usa el trozo (i mod N, j mod N) por coordenadas UV
    Image.fromarray(out_rgb, "RGB").save(os.path.join(out, "dirt_atlas.png"))
    rgba = np.dstack([rgb, alpha * 255]).astype(np.uint8)
    Image.fromarray(rgba, "RGBA").save(os.path.join(out, "..", "..", "preview.png"))
    print(f"atlas de {a.n}×{a.n} baldosas de {a.px}px en {os.path.normpath(out)}")


if __name__ == "__main__":
    main()
