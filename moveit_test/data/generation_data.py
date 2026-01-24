import os
import csv
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt

# =========================
# RÉGLAGES (à modifier)
# =========================

# Centre du dessin sur le tableau (dans VOS unités)
# Exemple : 0.00, 0.00 si repère est centré
CENTER_U = 0.00
CENTER_V = 0.00

# Taille du dessin (dans VOS unités)
# -> le dessin sera contenu dans un carré/rectangle de cette taille en gardant le ratio
# Exemple : 0.20 = 20 cm
DRAW_WIDTH  = 0.30
DRAW_HEIGHT = 0.30

# Échantillonnage : espacement entre points successifs 
POINT_SPACING = 0.005  # 2 mm 

# Seuils pour l'extraction des contours (Canny)
CANNY_T1 =  60
CANNY_T2 =150

# Simplification de contours : plus grand => moins de points (en pixels, avant mise à l'échelle)
APPROX_EPS_PIX = 8

# =========================
# UTILITAIRES
# =========================

def resample_polyline(points_uv, spacing):
    """
    Ré-échantillonne une polyligne (Nx2) pour obtenir des points espacés ~ spacing.
    """
    pts = np.asarray(points_uv, dtype=float)
    if len(pts) < 2:
        return pts

    # distances cumulées
    seg = pts[1:] - pts[:-1]
    seglen = np.linalg.norm(seg, axis=1)
    total = seglen.sum()
    if total <= 1e-12:
        return pts[:1]

    # positions cibles
    n = max(2, int(math.floor(total / spacing)) + 1)
    d_targets = np.linspace(0.0, total, n)

    # interpolation
    cum = np.concatenate([[0.0], np.cumsum(seglen)])
    out = []
    j = 0
    for dt in d_targets:
        while j < len(seglen) - 1 and cum[j+1] < dt:
            j += 1
        # dt dans segment j
        denom = seglen[j] if seglen[j] > 1e-12 else 1.0
        t = (dt - cum[j]) / denom
        p = pts[j] + t * (pts[j+1] - pts[j])
        out.append(p)
    return np.array(out)


def fit_points_to_box(points_xy, box_w, box_h):
    """
    Met à l'échelle (en gardant le ratio) et centre dans une boîte box_w x box_h.
    Entrée: points en coordonnées image "xy" (pixels)
    Sortie: points en coordonnées "uv" centrées sur (0,0) dans la boîte.
    """
    pts = np.asarray(points_xy, dtype=float)
    xmin, ymin = pts.min(axis=0)
    xmax, ymax = pts.max(axis=0)
    w = max(1e-9, xmax - xmin)
    h = max(1e-9, ymax - ymin)

    # mise à l'échelle en gardant le ratio pour rentrer dans box_w x box_h
    s = min(box_w / w, box_h / h)

    # centre
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    pts_centered = pts - np.array([cx, cy])
    pts_scaled = pts_centered * s

    return pts_scaled


def image_to_paths(image_path):
    """
    Charge l'image, extrait les contours, renvoie une liste de contours,
    chaque contour étant un array Nx2 en pixels (x,y).
    """
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Impossible de lire l'image: {image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # petit flou pour stabiliser le Canny
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(gray, CANNY_T1, CANNY_T2)

    # contours
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    paths = []
    for c in contours:
        if len(c) < 10:
            continue

        # c: Nx1x2 => Nx2
        pts = c.reshape(-1, 2).astype(float)

        # simplification
        eps = APPROX_EPS_PIX
        approx = cv2.approxPolyDP(pts.astype(np.float32), eps, False)
        approx = approx.reshape(-1, 2).astype(float)

        if len(approx) < 2:
            continue
        paths.append(approx)

    # trier par "taille" décroissante pour dessiner d'abord les gros contours
    paths.sort(key=lambda p: -len(p))
    return paths


def paths_to_csv_points(paths_pix, out_csv, center_u, center_v, box_w, box_h, spacing):
    """
    Convertit des contours pixels en points (u,v,pen) dans le repère final.
    - pen=0 pour déplacement
    - pen=1 pour tracer
    """
    # concat pour déterminer bounding global (pour scaling cohérent)
    all_pts = np.vstack(paths_pix)
    pts_fit = fit_points_to_box(all_pts, box_w, box_h)

    # On doit appliquer la même transformation à chaque contour :
    # on re-calcule les mêmes paramètres de scaling/centrage qu'utilisé dans fit_points_to_box
    xmin, ymin = all_pts.min(axis=0)
    xmax, ymax = all_pts.max(axis=0)
    w = max(1e-9, xmax - xmin)
    h = max(1e-9, ymax - ymin)
    s = min(box_w / w, box_h / h)
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0

    def transform(pix_xy):
        p = np.asarray(pix_xy, dtype=float)
        p = p - np.array([cx, cy])
        p = p * s
        # IMPORTANT: l'axe Y image est vers le bas, souvent on veut Y vers le haut => inversion
        p[:, 1] *= -1.0
        # translation vers le centre du tableau
        p[:, 0] += center_u
        p[:, 1] += center_v
        return p

    rows = []
    preview_segments = []  # liste de (Nx2, pen)

    for path in paths_pix:
        uv = transform(path)

        # ré-échantillonnage régulier
        uv_rs = resample_polyline(uv, spacing)
        if len(uv_rs) < 2:
            continue

        # pen up vers le premier point
        rows.append((uv_rs[0, 0], uv_rs[0, 1], 0))
        # puis pen down sur tout le contour
        for p in uv_rs:
            rows.append((p[0], p[1], 1))
        # pen up à la fin (optionnel mais propre)
        rows.append((uv_rs[-1, 0], uv_rs[-1, 1], 0))

        preview_segments.append((uv_rs, 1))

    # écriture CSV
    with open(out_csv, "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["u", "v", "pen"])
        for (u, v, pen) in rows:
            wcsv.writerow([f"{u:.5f}", f"{v:.5f}", int(pen)])

    return rows


def preview(rows):
    """
    Prévisualisation simple :
    - segments pen=1 en traits
    - points pen=0 affichés aussi (déplacements)
    """
    pts = np.array([(r[0], r[1], r[2]) for r in rows], dtype=float)
    u = pts[:, 0]
    v = pts[:, 1]
    pen = pts[:, 2].astype(int)

    plt.figure()
    # tracer uniquement les points pen=1 en reliant les runs continus
    start = None
    for i in range(len(pts)):
        if pen[i] == 1 and start is None:
            start = i
        if (pen[i] == 0 or i == len(pts) - 1) and start is not None:
            end = i if pen[i] == 0 else i + 1
            plt.plot(u[start:end], v[start:end])
            start = None

    # afficher les points pen=0 (déplacements) en points
    idx_up = np.where(pen == 0)[0]
    if len(idx_up) > 0:
        plt.scatter(u[idx_up], v[idx_up], s=10)

    plt.gca().set_aspect("equal", adjustable="box")
    plt.title("Prévisualisation du dessin (pen=1 en lignes, pen=0 en points)")
    plt.xlabel("u")
    plt.ylabel("v")
    plt.show()


# =========================
# MAIN
# =========================

def main():
    image_path = "moveit_test/data/input_insa.png"      # <-- mettre ici image
    out_csv = "moveit_test/data/output.csv"

    paths = image_to_paths(image_path)
    if len(paths) == 0:
        raise RuntimeError("Aucun contour détecté. Essayez d'ajuster CANNY_T1/CANNY_T2 ou l'image.")

    rows = paths_to_csv_points(
        paths_pix=paths,
        out_csv=out_csv,
        center_u=CENTER_U,
        center_v=CENTER_V,
        box_w=DRAW_WIDTH,
        box_h=DRAW_HEIGHT,
        spacing=POINT_SPACING
    )

    print(f"CSV généré: {out_csv} ({len(rows)} lignes)")
    preview(rows)


if __name__ == "__main__":
    main()