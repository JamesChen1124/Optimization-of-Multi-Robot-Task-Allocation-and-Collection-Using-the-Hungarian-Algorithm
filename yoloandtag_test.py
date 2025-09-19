import os, time, argparse
os.environ.setdefault("OMP_NUM_THREADS", "4")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO
from pupil_apriltags import Detector


# ---- Drawing helpers ---------------------------------------------------------
def draw_yolo(bgr, yolo_res):
    if yolo_res is None:
        return bgr
    names = yolo_res.names if hasattr(yolo_res, "names") else {}
    for i in range(len(yolo_res.boxes)):
        xyxy = yolo_res.boxes.xyxy[i].cpu().numpy().astype(int)
        cls  = int(yolo_res.boxes.cls[i].item())
        conf = float(yolo_res.boxes.conf[i].item())
        x1, y1, x2, y2 = xyxy
        cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{names.get(cls, str(cls))} {conf:.2f}"
        cv2.putText(bgr, label, (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return bgr


def draw_tags(bgr, tags):
    if tags is None:
        return bgr
    for t in tags:
        corners = t.corners.astype(int)
        cv2.polylines(bgr, [corners], True, (255, 0, 0), 2)
        c = tuple(np.round(t.center).astype(int))
        cv2.circle(bgr, c, 3, (255, 0, 0), -1)
        cv2.putText(bgr, f"id:{t.tag_id}", (c[0] + 6, c[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)
    return bgr


# ---- Color tweak (optional) targeted at yellow-green hues --------------------
def green_boost_bgr(bgr, h_lo=35, h_hi=85, h_shift=-6, s_mul=1.15, v_mul=1.05):
    """Selective HSV tweak for tennis-ball hues: shift hue slightly toward green,
       add a bit of saturation/brightness. Only applied to YOLO input if enabled."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0].astype(np.int16)
    s = hsv[:, :, 1].astype(np.float32)
    v = hsv[:, :, 2].astype(np.float32)

    mask = (h >= h_lo) & (h <= h_hi)
    h[mask] = (h[mask] + h_shift) % 180
    s[mask] = np.clip(s[mask] * s_mul, 0, 255)
    v[mask] = np.clip(v[mask] * v_mul, 0, 255)

    hsv[:, :, 0] = h.astype(np.uint8)
    hsv[:, :, 1] = s.astype(np.uint8)
    hsv[:, :, 2] = v.astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


# ---- Manual white-balance helper (grey-world on center ROI) ------------------
def wb_from_center_roi_set(picam2, bgr, roi_frac=0.30):
    h, w, _ = bgr.shape
    x0 = int(w * (0.5 - roi_frac / 2)); x1 = int(w * (0.5 + roi_frac / 2))
    y0 = int(h * (0.5 - roi_frac / 2)); y1 = int(h * (0.5 + roi_frac / 2))
    roi = bgr[y0:y1, x0:x1]
    B, G, R = roi.mean(axis=(0, 1))  # BGR order
    R_gain = float(G / max(R, 1e-6))
    B_gain = float(G / max(B, 1e-6))
    picam2.set_controls({"AwbEnable": False, "ColourGains": (R_gain, B_gain)})
    return (x0, y0, x1, y1), (R_gain, B_gain)


def main():
    ap = argparse.ArgumentParser()
    # Model / thresholds
    ap.add_argument("--model", type=str, required=True, help="Path to YOLO ONNX")
    ap.add_argument("--imgsz", type=int, default=640, help="YOLO input size (must match ONNX)")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO conf threshold")
    ap.add_argument("--iou",  type=float, default=0.45, help="YOLO NMS IoU")

    # Camera & color
    ap.add_argument("--width",  type=int, default=1640, help="Camera width")
    ap.add_argument("--height", type=int, default=1232, help="Camera height")
    ap.add_argument("--format", type=str, default="BGR888", choices=["BGR888", "RGB888"])
    ap.add_argument("--wb", type=str, default="awb", choices=["awb", "manual"],
                    help="awb: auto white balance (stable mode); manual: ROI-based lock")
    ap.add_argument("--awb-mode", type=int, default=4, help="AWB mode if wb=awb (0=Auto,4=Daylight)")
    ap.add_argument("--wb-roi", type=float, default=0.30, help="Center ROI fraction for manual WB")
    ap.add_argument("--rb-swap", type=int, default=0, help="Swap R/B channels (1=on)")

    # AprilTag
    ap.add_argument("--family", type=str, default="tag36h11")
    ap.add_argument("--tag-size", type=float, default=0.0, help="Meters (0 disables pose)")
    ap.add_argument("--fx", type=float, default=0.0)
    ap.add_argument("--fy", type=float, default=0.0)
    ap.add_argument("--cx", type=float, default=0.0)
    ap.add_argument("--cy", type=float, default=0.0)
    ap.add_argument("--atag-decimate", type=float, default=2.0,
                    help="AprilTag quad_decimate")
    ap.add_argument("--atag-w", type=int, default=960,
                    help="Resize width for AprilTag processing (0=no resize)")

    # Scheduling & display
    ap.add_argument("--ratio", type=int, default=2,
                    help="YOLO frames per cycle; Tag runs on the remaining 1 frame (>=1)")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--save-dir", type=str, default="/home/pi",
                    help="Directory for saved frames via 's' key")
    # Optional tennis-green fix for YOLO input only
    ap.add_argument("--greenfix", type=int, default=0, help="Enable tennis green tweak (1=on)")
    args = ap.parse_args()

    # YOLO
    yolo = YOLO(args.model)

    # AprilTag detector
    detector = Detector(
        families=args.family,
        nthreads=2,
        quad_decimate=args.atag_decimate,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25
    )

    # Camera config
    picam2 = Picamera2()
    controls = {"FrameRate": 30, "AeEnable": True}
    if args.wb == "awb":
        controls.update({"AwbEnable": True, "AwbMode": args.awb_mode})
    else:
        controls.update({"AwbEnable": False})  # will set ColourGains after first frame

    fmt = args.format
    cfg = picam2.create_video_configuration(
        main={"size": (args.width, args.height), "format": fmt},
        controls=controls
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(0.25)

    # Pose flag
    need_pose = (args.tag_size > 0.0 and args.fx > 0 and args.fy > 0 and args.cx > 0 and args.cy > 0)
    base_cam_params = (args.fx, args.fy, args.cx, args.cy) if need_pose else None

    # State
    last_yolo_res = None
    last_tags = None
    tick = 0
    yolo_t0 = time.time(); yolo_runs = 0
    at_t0   = time.time(); at_runs   = 0
    wb_box = None
    manual_override = (args.wb == "manual")
    rb_swap = bool(args.rb_swap)
    greenfix = bool(args.greenfix)

    while True:
        # --- Capture ----------------------------------------------------------
        frame = picam2.capture_array()  # BGR888 or RGB888 depending on cfg
        if fmt == "RGB888":
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            bgr = frame

        # Optional R/B swap (to fix sensor/ISP channel swap if observed)
        if rb_swap:
            bgr = bgr[:, :, [2, 1, 0]]

        # Manual WB: lock on first frame if requested
        if manual_override and wb_box is None:
            wb_box, gains = wb_from_center_roi_set(picam2, bgr, roi_frac=args.wb_roi)

        # --- Inference scheduling --------------------------------------------
        do_yolo = (tick % (args.ratio + 1) < args.ratio)  # e.g., ratio=2 => YOLO on 2/3 frames
        if do_yolo:
            # Optionally adjust color just for YOLO input (not for display)
            yolo_in = bgr
            if greenfix:
                yolo_in = green_boost_bgr(yolo_in)

            res = yolo.predict(
                yolo_in, imgsz=args.imgsz, conf=args.conf, iou=args.iou,
                device='cpu', verbose=False
            )[0]
            last_yolo_res = res
            yolo_runs += 1
        else:
            # AprilTag: optionally resize for speed
            gray_for_tag = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            cam_params = base_cam_params
            if args.atag_w and (args.atag_w < gray_for_tag.shape[1]):
                scale = args.atag_w / gray_for_tag.shape[1]
                gray_small = cv2.resize(gray_for_tag, (int(gray_for_tag.shape[1]*scale),
                                                       int(gray_for_tag.shape[0]*scale)),
                                        interpolation=cv2.INTER_AREA)
                if need_pose:
                    fx, fy, cx, cy = base_cam_params
                    cam_params = (fx*scale, fy*scale, cx*scale, cy*scale)
                tags = detector.detect(gray_small,
                                       estimate_tag_pose=need_pose,
                                       camera_params=cam_params,
                                       tag_size=args.tag_size if need_pose else None)
                # scale corners/centers back for drawing
                for t in tags:
                    t.corners /= scale
                    t.center  /= scale
            else:
                tags = detector.detect(gray_for_tag,
                                       estimate_tag_pose=need_pose,
                                       camera_params=cam_params,
                                       tag_size=args.tag_size if need_pose else None)
            last_tags = tags
            at_runs += 1

        # --- Compose output ---------------------------------------------------
        out = bgr.copy()
        out = draw_yolo(out, last_yolo_res)
        out = draw_tags(out, last_tags)

        # HUD
        now = time.time()
        yolo_fps = yolo_runs / max(1e-6, (now - yolo_t0))
        at_fps   = at_runs   / max(1e-6, (now - at_t0))
        wb_text = "AWB" if not manual_override else "WB:MANUAL"
        swap_text = "RBswap:ON" if rb_swap else "RBswap:OFF"
        gfix_text = "Gfix:ON" if greenfix else "Gfix:OFF"
        info = f"YOLO {yolo_fps:.1f} FPS | AprilTag {at_fps:.1f} FPS | {wb_text} | {swap_text} | {gfix_text}"
        cv2.putText(out, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2, cv2.LINE_AA)

        # Show WB ROI box when manual
        if manual_override and wb_box is not None:
            x0, y0, x1, y1 = wb_box
            cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 255), 1)

        # --- UI / keys --------------------------------------------------------
        if args.show:
            cv2.imshow("Pi5 YOLO + AprilTag", out)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:   # ESC
                break
            elif key == ord('0'):
                # back to AWB Daylight
                manual_override = False
                picam2.set_controls({"AwbEnable": True, "AwbMode": 4})
            elif key == ord('c'):
                # recalibrate manual WB (place white/gray at center)
                manual_override = True
                wb_box, gains = wb_from_center_roi_set(picam2, bgr, roi_frac=args.wb_roi)
            elif key == ord('x'):
                # toggle RB swap to diagnose/solve blue<->red issue
                rb_swap = not rb_swap
            elif key == ord('g'):
                # toggle greenfix for YOLO input
                greenfix = not greenfix
            elif key == ord('s'):
                # save current frame
                ts = int(time.time())
                path = os.path.join(args.save_dir, f"frame_{ts}.jpg")
                cv2.imwrite(path, out)
                print(f"Saved: {path}")

        tick += 1

    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
