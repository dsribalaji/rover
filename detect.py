#!/usr/bin/env python3
"""Minimal camera access logger.

This script opens a camera device, reads frames continuously,
and logs basic information (timestamp, frame number, size,
mean brightness and approximate fps) to the terminal.

Usage examples:
  python3 detect.py
  python3 detect.py --device 0 --interval 0.5
"""

import argparse
import datetime
import sys
import signal
import time

import cv2
import numpy as np
import threading
import queue
from typing import Dict, List, Tuple


def _signal_handler(sig, frame):
    print("\nReceived interrupt — exiting cleanly")
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="Camera access logger")
    parser.add_argument("--device", "-d", type=int, default=0, help="camera device index (default: 0)")
    parser.add_argument(
        "--interval",
        "-i",
        type=float,
        default=0.0,
        help="minimum seconds between printed log lines (0 = every frame)",
    )
    parser.add_argument("--model", "-m", default="yolov8n.pt", help="path to yolov8 model (default: yolov8n.pt)")
    parser.add_argument("--conf", "-c", type=float, default=0.25, help="confidence threshold")
    parser.add_argument("--skip", "-s", type=int, default=0, help="skip N frames between inferences (0 = every frame)")
    parser.add_argument("--width", type=int, default=640, help="capture width (px)")
    parser.add_argument("--height", type=int, default=360, help="capture height (px)")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size (px)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera device {args.device}", file=sys.stderr)
        sys.exit(2)

    print(f"Opened camera device {args.device}")

    # attempt to load YOLO model
    # configure capture resolution to reduce CPU load
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    try:
        from ultralytics import YOLO
    except Exception as e:
        print("ERROR: ultralytics package is required for YOLO inference. Install with: pip install ultralytics", file=sys.stderr)
        print("Exception:", e, file=sys.stderr)
        cap.release()
        sys.exit(3)

    try:
        model = YOLO(args.model)
    except Exception as e:
        print(f"ERROR: failed to load model '{args.model}': {e}", file=sys.stderr)
        cap.release()
        sys.exit(4)

    # select device and enable half precision on CUDA for speed if possible
    try:
        import torch as _torch
        device = "cuda" if _torch.cuda.is_available() else "cpu"
    except Exception:
        device = "cpu"

    try:
        # move model to device if supported
        try:
            model.to(device)
        except Exception:
            pass
        if device == "cuda":
            try:
                model.model.half()
            except Exception:
                pass
    except Exception:
        pass

    print(f"Loaded model: {args.model} on {device}")
    frame_count = 0
    start_time = time.time()
    last_log_time = 0.0
    fps_acc_count = 0
    fps_acc_time = time.time()

    # Use a single-slot queue so we always infer the latest frame
    frame_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)
    stop_event = threading.Event()

    def _capture_loop():
        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # downscale to capture resolution to reduce processing
            if args.width and args.height:
                if frame.shape[1] != args.width or frame.shape[0] != args.height:
                    frame = cv2.resize(frame, (args.width, args.height), interpolation=cv2.INTER_LINEAR)

            try:
                frame_queue.put(frame, block=False)
            except queue.Full:
                # drop older frame, keep newest
                try:
                    _ = frame_queue.get_nowait()
                except Exception:
                    pass
                try:
                    frame_queue.put(frame, block=False)
                except Exception:
                    pass

    # exponential moving average for fps
    ema_fps = 0.0
    alpha = 0.15

    def _inference_loop():
        nonlocal frame_count, fps_acc_count, last_log_time, fps_acc_time, ema_fps
        while not stop_event.is_set():
            try:
                frame = frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            frame_count += 1
            t0 = time.time()

            # run model inference (pass imgsz for speed/accuracy tradeoff)
            try:
                res = model(frame, imgsz=args.imgsz, conf=args.conf)[0]
            except Exception as e:
                print(f"WARNING: inference error: {e}", file=sys.stderr)
                continue

            t1 = time.time()
            infer_time = t1 - t0
            fps_now = 1.0 / infer_time if infer_time > 0 else 0.0
            if ema_fps == 0.0:
                ema_fps = fps_now
            else:
                ema_fps = ema_fps * (1 - alpha) + fps_now * alpha

            # parse detections
            detections_summary: List[Tuple[str, float]] = []
            boxes = getattr(res, "boxes", None)
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    try:
                        cls_id = int(box.cls[0]) if hasattr(box, "cls") and len(box.cls) > 0 else int(box.cls)
                    except Exception:
                        try:
                            cls_id = int(box.cls)
                        except Exception:
                            cls_id = -1

                    try:
                        confv = float(box.conf[0]) if hasattr(box, "conf") and len(box.conf) > 0 else float(box.conf)
                    except Exception:
                        try:
                            confv = float(box.conf)
                        except Exception:
                            confv = 0.0

                    name = model.names.get(cls_id, str(cls_id)) if hasattr(model, "names") else str(cls_id)
                    detections_summary.append((name, confv))

            # summarize detections
            counts: Dict[str, int] = {}
            top_per_class: Dict[str, float] = {}
            for name, confv in detections_summary:
                counts[name] = counts.get(name, 0) + 1
                top_per_class[name] = max(top_per_class.get(name, 0.0), confv)

            if counts:
                dets_str = ", ".join([f"{k}:{v}(@{top_per_class[k]:.2f})" for k, v in counts.items()])
            else:
                dets_str = "none"

            now = time.time()
            elapsed = now - start_time

            # print a compact log line
            print(f"{datetime.datetime.now().isoformat(sep=' ', timespec='milliseconds')} | frame={frame_count} | size={args.width}x{args.height} | fps={ema_fps:.2f} | detections={dets_str} | elapsed={elapsed:.2f}s")

            # small sleep to yield
            time.sleep(0.001)

    # start threads
    cap_thread = threading.Thread(target=_capture_loop, daemon=True)
    infer_thread = threading.Thread(target=_inference_loop, daemon=True)
    cap_thread.start()
    infer_thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        stop_event.set()
        cap_thread.join(timeout=1.0)
        infer_thread.join(timeout=1.0)
        cap.release()
        print("Camera released")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _signal_handler)
    main()