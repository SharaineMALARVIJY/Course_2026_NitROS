import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import yaml
from PIL import Image


def get_resolution_origin(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    return data["resolution"], data["origin"]


def open_image(image_path, rotate=False):
    img = Image.open(image_path)
    if rotate:
        img= np.rot90(img)
    return np.array(img, dtype=np.float32)


def pix_to_coord(pix_x, pix_y, resolution, origin, height):
    x = origin[0] + pix_x * resolution
    y = origin[1] + (height - pix_y) * resolution  # origine image en haut à gauche
    return x, y


def to_vect(points):
    if len(points) % 2 != 0:
        return None

    vects = []

    for i in range(0, len(points), 2):
        a = points[i]
        b = points[i + 1]

        dx = b[0] - a[0]
        dy = b[1] - a[1]
        theta = float(np.arctan2(dy, dx))

        vects.append([a[0], a[1], theta])

    return vects


def reverse_vect(vects):
    reversed_vects = []

    for vect in vects:
        x, y, theta = vect
        reversed_vects.append([x, y, theta + np.pi])

    return reversed_vects


def print_course(vects, reversed_vects):
    if not vects:
        print("Aucun vecteur à afficher.")
        return

    print(f"    p0: {vects[0]}")
    print("\n")

    for i in range(len(vects)):
        print(f"    p{i}: true")

    print("\n")

    for i in range(1, len(vects)):
        print(f"    p{i}: {vects[i]}")

    print("\n")

    for i in range(1, len(reversed_vects)):
        print(f"    p{i}: {reversed_vects[i]}")


def onclick(event, resolution, origin, height):
    if event.xdata is None or event.ydata is None:
        return

    px = int(event.xdata)
    py = int(event.ydata)

    real_point = pix_to_coord(px, py, resolution, origin, height)
    x_y_reals.append(real_point)

    vects = to_vect(x_y_reals)


yaml_path = "map_etage_2.yaml"
image_path = "map_etage_2_cleaned.pgm"

image = open_image(image_path, True)
resolution, origin = get_resolution_origin(yaml_path)
height, width = image.shape

x_y_reals = []

fig, ax = plt.subplots()
ax.imshow(image, cmap="gray")

cid = fig.canvas.mpl_connect(
    "button_press_event",
    lambda event: onclick(event, resolution, origin, height)
)

plt.show()

vects = to_vect(x_y_reals)
reversed_vect = reverse_vect(vects)
print_course(vects, reversed_vect)
