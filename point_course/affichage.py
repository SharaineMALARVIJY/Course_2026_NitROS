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
        img = np.rot90(img)
    return np.array(img, dtype=np.float32)


def pix_to_coord(pix_x, pix_y, resolution, origin, height):
    x = origin[0] + pix_x * resolution
    y = origin[1] + (height - pix_y) * resolution
    return x, y


def coord_to_pix(x, y, resolution, origin, height):
    pix_x = (x - origin[0]) / resolution
    pix_y = height - ((y - origin[1]) / resolution)
    return pix_x, pix_y


# ========================
# FICHIERS
# ========================
yaml_path = "final.yaml"
image_path = "final.pgm"

image = open_image(image_path, False)
resolution, origin = get_resolution_origin(yaml_path)
height, width = image.shape


# ========================
# POINTS DONNÉS
# ========================

# test 3
points = [
    [-0.46399999999999997, -1.0099999999999998, -0.5880026035475677],
    [1.036, -1.81, -0.4324077755705374],
    [2.4360000000000004, 2.69, 2.5127963671743596],
    [-2.264, 0.3400000000000003, -0.8441539861131713],
]

# test 4
points = [
    [-0.142, -0.050, 1.562],
    [-3.425, 5.0680000000000005, 2.9441970937399122],
    [-6.375, 2.8680000000000003, -1.4801364395941519],
    [-3.075, -1.032, -1.4536875822280317],
    [-0.142, -0.050, 1.562],
]

# # test 4 reverse
# points = [
#     [-0.045, 0.005, -1.543],   # p0
#     [-2.9749999999999996, -0.9820000000000002, 1.543775877607632],   # p1
#     [-6.425, 2.1180000000000003, 1.5957911204138167],# p2
#     [-3.175, 5.268, 0.01817981507297854], # p3
#     [-0.4249999999999998, 2.218, -1.6359214901292822],  # p4
# ]

# final
points = [
    [-0.626, 0.203, 1.648],
    [-1.0219999999999994, 2.367, 1.8673421358645974],
    [-5.772, 3.317000000000001, -2.7868870015788527],
    [-5.572, -1.783, -0.03224688243525381],
    [-0.7719999999999994, -2.133, 1.5707963267948966],
]

#final reverse
points = [
    [-1.207, -1.061, -1.63736449057072],
    [-5.372, -1.783, 3.0258334358689822],
    [-7.122, 0.9170000000000007, 1.6092389168160846],
    [-1.072, 2.5170000000000003, -1.6162196062164735],
    [-1.072, -2.9829999999999997, -1.6447353644528366],
]

labels = ["p0", "p1", "p2", "p3", "p4"]


fig, ax = plt.subplots()
ax.imshow(image, cmap="gray")

arrow_length_m = 0.5

for i, (x, y, theta) in enumerate(points):
    px, py = coord_to_pix(x, y, resolution, origin, height)

    ax.plot(px, py, "ro", markersize=6)
    ax.text(px + 5, py + 5, labels[i], color="red", fontsize=10)

    dx_m = arrow_length_m * np.cos(theta)
    dy_m = arrow_length_m * np.sin(theta)

    px2, py2 = coord_to_pix(x + dx_m, y + dy_m, resolution, origin, height)

    ax.arrow(
        px, py,
        px2 - px, py2 - py,
        head_width=5,
        head_length=5,
        fc="blue",
        ec="blue"
    )

ax.set_title("Affichage des points et orientations")
plt.show()
