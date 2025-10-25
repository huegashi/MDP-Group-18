#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time
import subprocess
import requests

# --- Config: set your API address here ---
API_IP = "127.0.0.1"   # e.g. "192.168.1.23"
API_PORT = 5000

def capture_with_libcamera(output_path: str,
                           brightness: float = 0.5,
                           contrast: float = 0.5,
                           shutter_us: int | None = None,
                           gain: float | None = None,
                           awb: str = "auto",
                           metering: str = "centre",
                           quality: int = 90) -> None:
    """
    Captures one image using libcamera-still to the given path.
    Only uses a few sane defaults; extend flags as needed.
    """
    cmd = [
        "libcamera-still",
        "-t", "500",        # 0.5s capture time
        "-o", output_path,
        "--brightness", str(brightness),
        "--contrast",   str(contrast),
        "--metering",   metering,
        "--quality",    str(quality),   
    ]
    if shutter_us is not None:
        cmd += ["--shutter", str(shutter_us)]
    if gain is not None:
        cmd += ["--gain", str(gain)]
    if awb:
        cmd += ["--awb", awb]

    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print("ERROR: libcamera-still not found. Install it or use --from-file.", file=sys.stderr)
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"ERROR: libcamera-still failed (code {e.returncode}).", file=sys.stderr)
        sys.exit(1)

def post_image(url: str, path: str) -> dict:
    with open(path, "rb") as f:
        files = {"file": (os.path.basename(path), f, "image/jpeg")}
        r = requests.post(url, files=files, timeout=10)
    if r.status_code != 200:
        raise RuntimeError(f"API returned HTTP {r.status_code}: {r.text}")
    try:
        return r.json()
    except json.JSONDecodeError:
        raise RuntimeError(f"API returned non-JSON: {r.text[:200]}")

def main():
    parser = argparse.ArgumentParser(description="Snap images for 10 seconds and call image-rec API.")
    parser.add_argument("--api-ip", default=API_IP)
    parser.add_argument("--api-port", type=int, default=API_PORT)
    parser.add_argument("--obstacle", default="obs0")
    parser.add_argument("--signal", default="C", choices=["L","R","C"])
    parser.add_argument("--out-dir", default=".")
    parser.add_argument("--keep", action="store_true")
    args = parser.parse_args()

    url = f"http://{args.api_ip}:{args.api_port}/image"

    start_time = time.time()
    while time.time() - start_time < 10:   # run for ~10 seconds
        ts = int(time.time())
        filename = f"{ts}_{args.obstacle}_{args.signal}.jpg"
        out_path = os.path.join(args.out_dir, filename)

        # 1. capture
        capture_with_libcamera(out_path)

        # 2. send to API
        result = post_image(url, out_path)
        print("Result:", result)

        # 3. cleanup (optional)
        if not args.keep:
            try:
                os.remove(out_path)
            except OSError:
                pass

        # 4. wait a second before next capture
        time.sleep(1)


if __name__ == "__main__":
    main()
