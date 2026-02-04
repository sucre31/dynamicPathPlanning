# map_to_image.py
# Usage:
#   python map_to_image.py map1-1.txt out.png 256
#   python map_to_image.py map1-1.txt out.ppm 256
#   python map_to_image.py --batch 256

import sys

def read_map(path):
    with open(path, "r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f if line.strip() != ""]
    h = len(lines)
    w = len(lines[0]) if h > 0 else 0
    return lines, w, h

def color_of(ch):
    if ch == "S" or ch == "G":
        return (255, 0, 0)      # red
    if ch in ("1", "#", "X"):
        return (160, 160, 160)  # gray
    return (255, 255, 255)      # white

def write_ppm(lines, w, h, out_size, out_path):
    W = out_size
    H = out_size
    with open(out_path, "w", encoding="ascii") as f:
        f.write(f"P3\n{W} {H}\n255\n")
        for y in range(H):
            src_y = int(y * h / H)
            row = lines[src_y]
            for x in range(W):
                src_x = int(x * w / W)
                r, g, b = color_of(row[src_x])
                f.write(f"{r} {g} {b} ")
            f.write("\n")

def write_png_with_pillow(lines, w, h, out_size, out_path):
    try:
        from PIL import Image
    except Exception:
        return False
    W = out_size
    H = out_size
    img = Image.new("RGB", (W, H), (255, 255, 255))
    px = img.load()
    for y in range(H):
        src_y = int(y * h / H)
        row = lines[src_y]
        for x in range(W):
            src_x = int(x * w / W)
            px[x, y] = color_of(row[src_x])
    img.save(out_path)
    return True

def batch_generate(out_size):
    # map1-1 .. map3-3 -> png
    for i in range(1, 4):
        for j in range(1, 4):
            in_path = f"map{i}-{j}.txt"
            out_path = f"map{i}-{j}.png"
            lines, w, h = read_map(in_path)
            if w == 0 or h == 0:
                print(f"Skip (empty): {in_path}")
                continue
            ok = write_png_with_pillow(lines, w, h, out_size, out_path)
            if ok:
                print(f"Wrote {out_path}")
            else:
                out_ppm = out_path.replace(".png", ".ppm")
                print("Pillow not available. Writing PPM instead.")
                write_ppm(lines, w, h, out_size, out_ppm)
                print(f"Wrote {out_ppm}")

def main():
    if len(sys.argv) < 3:
        if len(sys.argv) == 2 and sys.argv[1] == "--batch":
            batch_generate(256)
            return
        print("Usage: python map_to_image.py <map.txt> <out.png|out.ppm> [out_size]")
        print("       python map_to_image.py --batch [out_size]")
        sys.exit(1)

    if sys.argv[1] == "--batch":
        out_size = int(sys.argv[2]) if len(sys.argv) >= 3 else 256
        batch_generate(out_size)
        return

    in_path = sys.argv[1]
    out_path = sys.argv[2]
    out_size = int(sys.argv[3]) if len(sys.argv) >= 4 else 256

    lines, w, h = read_map(in_path)
    if w == 0 or h == 0:
        print("Empty map.")
        return

    if out_path.lower().endswith(".png"):
        ok = write_png_with_pillow(lines, w, h, out_size, out_path)
        if not ok:
            print("Pillow not available. Writing PPM instead.")
            write_ppm(lines, w, h, out_size, out_path.replace(".png", ".ppm"))
    else:
        write_ppm(lines, w, h, out_size, out_path)

if __name__ == "__main__":
    main()
