# png_to_bin_scaled.py
from PIL import Image
import numpy as np

def png_to_bin(png_file, bin_file):
    # --- Open PNG without converting ---
    img = Image.open(png_file)
    data = np.array(img)
    print(f"Original image mode: {img.mode}")
    print(f"Image shape: {data.shape}")
    print(f"Sample pixels [10:15, 20:25]:\n{data[10:15, 20:25]}")

    # --- Scale to 0-255 ---
    data_scaled = (data.astype(np.float32) - data.min()) / (data.max() - data.min()) * 255
    data_uint16 = data_scaled.astype(np.uint16)

    # --- Debug: print a small patch ---
    print(f"Scaled sample [10:15, 20:25]:\n{data_uint16[10:15, 20:25]}")

    # --- Save as row-major binary ---
    data_uint16.tofile(bin_file)
    print(f"Saved {data_uint16.shape[0]}x{data_uint16.shape[1]} PNG to binary: {bin_file}")

if __name__ == "__main__":
    png_to_bin("heightmap.png", "heightmap.bin")
