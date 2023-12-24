import math


PI = 3.1415926

D30 = PI / 6.
D90 = PI / 2.
D120 = 2. * PI / 3.
D150 = 5. * PI / 6.
D210 = 7. * PI / 6.
D240 = 4. * PI / 3.
D270 = 3. * PI / 2.
D330 = 11. * PI / 6.
D360 = 2 * PI

def r_value(angle):
    projection = abs(math.sin(angle - D240))
    if D30 <= angle <= D90:
        projection = 0.
    elif D90 <= angle < D120:
        projection -= g_value(angle)
    elif 0 <= angle < D30:
        projection -= b_value(angle)
    return projection

def g_value(angle):
    projection = abs(math.sin(angle - D120))
    if D270 <= angle <= D330:
        projection = 0.
    elif D240 <= angle < D270:
        projection -= r_value(angle)
    elif D330 <= angle < D360:
        projection -= b_value(angle)
    return projection

def b_value(angle):
    projection = abs(math.sin(angle))
    if D150 <= angle <= D210:
        projection = 0.
    elif D120 <= angle < D150:
        projection -= g_value(angle)
    elif D210 <= angle < D240:
        projection -= r_value(angle)
    return projection

def rgb_for_angle(angle):
    return r_value(angle), g_value(angle), b_value(angle)


def estimate_size(pixels):
    start, stop = None, None
    size = len(pixels)
    for idx, pixel in enumerate(pixels):
        if idx and sum(pixel) > 0. and sum(pixels[idx - 1]) == 0.:
            start = idx
        if idx < size - 1 and sum(pixel) > 0. and sum(pixels[idx + 1]) == 0.:
            stop = idx
    return stop - start + 1


def normalize_color(rgb):
    total = sum(rgb)
    r, g, b = rgb
    return r / total, g / total, b / total


def l2_norm(rgb):
    r, g, b = rgb
    return math.sqrt(r ** 2 + g ** 2 + b ** 2)


def find_angle(rgb):
    result = None
    error = None
    r, g, b = normalize_color(rgb)
    for i in range(100):
        angle = i * 2 * PI / 100.
        etalon_r, etalon_g, etalon_b = normalize_color(rgb_for_angle(angle))
        err = l2_norm((r - etalon_r, g - etalon_g, b - etalon_b))
        if error is None or err < error:
            error = err
            result = angle
    result = 2 * PI - result
    return result

def get_projection_relative_size(angle):
    while (angle > PI / 6.):
        angle -= PI / 3.
    return math.cos(angle)


def solve(fov, pixels):
    pixel_size = estimate_size(pixels)
    r_acc = sum([x[0] for x in pixels])
    g_acc = sum([x[1] for x in pixels])
    b_acc = sum([x[2] for x in pixels])
    norm_rgb = normalize_color((r_acc, g_acc, b_acc))
    angle = find_angle(norm_rgb)
    relative_size = get_projection_relative_size(angle)
    projection_size = 2 * math.sin(PI / 3.) * relative_size
    spinner_angle_size = fov * pixel_size / len(pixels)
    distance = abs(projection_size / math.tan(0.5 * spinner_angle_size))
    x = abs(math.cos(angle)) * distance
    y = abs(math.sin(angle)) * distance
    x = -1 * x if angle < PI / 2. or angle > 3. * PI / 2. else x
    y = -1 * y if angle > 180 else x
    return x, y, angle

if __name__ == "__main__":
    cam_resolution, cam_fov = input().split()
    cam_resolution, cam_fov = int(cam_resolution), float(cam_fov)
    pixels = []
    for i in range(cam_resolution):
        color = [float(x) for x in input().split()]
        pixels.append(color)
    x, y, teta = solve(cam_fov, pixels)
    print(f"{x} {y} {teta}")
