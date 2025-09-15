from PIL import Image
import struct
import os

# Batch-convert tools/f1.jpg..tools/f16.jpg -> spiffs_image/1.bin..16.bin
os.makedirs('spiffs_image', exist_ok=True)

for n in range(1, 17):
    src = f'tools/f ({n}).jpg'
    dst = f'spiffs_image/{n}.bin'
    if not os.path.exists(src):
        print(f"Skipping {src}: not found")
        continue
    img = Image.open(src)
    img = img.convert('RGB')
    img = img.resize((480, 320))
    with open(dst, 'wb') as f:
        for y in range(img.height):
            for x in range(img.width):
                r, g, b = img.getpixel((x, y))
                rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
                # write little-endian 16-bit value (matches typical ESP newlib packing)
                f.write(struct.pack('<H', rgb565))
    print(f'Wrote {dst} ({os.path.getsize(dst)} bytes)')
