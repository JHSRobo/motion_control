import hid
import time
import sys
import math

VENDOR_ID = 0x054C

def get_stick_data(x_raw, y_raw, deadzone=0.12):
    dir_x_norm = (x_raw - 127.5) / 127.5
    dir_y_norm = -(y_raw - 127.5) / 127.5

    mag_norm = math.hypot(dir_x_norm, dir_y_norm)
    if mag_norm < deadzone:
        return 50.0, 50.0, 0.0, 0.0

    mag = min(1.0, mag_norm) * 100.0
    
    dir_x = ((dir_x_norm + 1.0) / 2.0) * 100.0
    dir_y = ((dir_y_norm + 1.0) / 2.0) * 100.0

    radians = math.atan2(dir_y_norm, dir_x_norm)
    degrees = math.degrees(radians)
    if degrees < 0:
        degrees += 360.0

    return dir_x, dir_y, degrees, mag

def parse_controller_data(data):
    if len(data) < 10:
        return None 

    l_dir_x, l_dir_y, l_deg, l_mag = get_stick_data(data[1], data[2])
    r_dir_x, r_dir_y, r_deg, r_mag = get_stick_data(data[3], data[4])

    dpad_val = data[5] & 0x0F
    d1_up    = 100.0 if dpad_val in (0, 1, 7) else 0.0
    d2_right = 100.0 if dpad_val in (1, 2, 3) else 0.0
    d3_down  = 100.0 if dpad_val in (3, 4, 5) else 0.0
    d4_left  = 100.0 if dpad_val in (5, 6, 7) else 0.0

    s1_square   = 100.0 if bool(data[5] & (1 << 4)) else 0.0
    x1_cross    = 100.0 if bool(data[5] & (1 << 5)) else 0.0
    c1_circle   = 100.0 if bool(data[5] & (1 << 6)) else 0.0
    t1_triangle = 100.0 if bool(data[5] & (1 << 7)) else 0.0

    l1 = 100.0 if bool(data[6] & (1 << 0)) else 0.0
    r1 = 100.0 if bool(data[6] & (1 << 1)) else 0.0
    share = 100.0 if bool(data[6] & (1 << 4)) else 0.0
    options = 100.0 if bool(data[6] & (1 << 5)) else 0.0
    l3 = 100.0 if bool(data[6] & (1 << 6)) else 0.0
    r3 = 100.0 if bool(data[6] & (1 << 7)) else 0.0

    l2 = (data[8] / 255.0) * 100.0
    r2 = (data[9] / 255.0) * 100.0

    display_output = []
    display_output.append("=== ROV CONTROL VECTORS ===")
    display_output.append(f"Forward/backward vector: <{l_dir_y:5.1f}, {l_deg:5.1f}°, {l_mag:5.1f}>")
    display_output.append(f"Left/right vector: <{l_dir_x:5.1f}, {l_deg:5.1f}°, {l_mag:5.1f}>")
    display_output.append(f"Angular up/down vector: <{r_dir_y:5.1f}, {r_deg:5.1f}°, {r_mag:5.1f}>")
    display_output.append(f"Angular left/right: <{r_dir_x:5.1f}, {r_deg:5.1f}°, {r_mag:5.1f}>")

    if c1_circle == 0.0:
        display_output.append(f"Vertical Vector Down: <{l2:5.1f}>")
        display_output.append(f"Vertical Vector Up: <{r2:5.1f}>")
    elif c1_circle == 100.0:
        display_output.append(f"Roll vector Left: <{l2:5.1f}>")
        display_output.append(f"Roll vector Right: <{r2:5.1f}>")

    sys.stdout.write("\033[H\033[J")
    sys.stdout.write("\n".join(display_output) + "\n")
    sys.stdout.flush()

    return {
        "l_x": l_dir_x, "l_y": l_dir_y, "l_deg": l_deg, "l_mag": l_mag,
        "r_x": r_dir_x, "r_y": r_dir_y, "r_deg": r_deg, "r_mag": r_mag,
        "d1": d1_up, "d2": d2_right, "d3": d3_down, "d4": d4_left,
        "s1": s1_square, "x1": x1_cross, "c1": c1_circle, "t1": t1_triangle,
        "l1": l1, "r1": r1, "l2": l2, "r2": r2,
        "share": share, "opt": options, "l3": l3, "r3": r3
    }

def main():
    device_info = None
    for device_dict in hid.enumerate():
        if device_dict['vendor_id'] == VENDOR_ID:
            device_info = device_dict
            break

    if not device_info:
        print("didnt connect or something")
        return

    print(f"Connected to: {device_info['product_string']}")
  
    time.sleep(1)

    device = hid.device()
    device.open_path(device_info['path'])
    device.set_nonblocking(True)

    try:
        while True:
            data = device.read(64)
            if data:
                parse_controller_data(data)
            time.sleep(0.01)
    except KeyboardInterrupt:
     
        device.close()

if __name__ == "__main__":
    main()
