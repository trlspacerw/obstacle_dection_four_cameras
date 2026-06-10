# Obstacle Avoidance on 4 Cameras (DJI-style)

This document walks through, step by step, how the DJI-style obstacle
avoidance system in `obstacle_avoidance_four_cameras.py` was built on top
of the existing 4-camera object detection script.

---

## 1. Starting point

The repo already contained `detect_four_cameras.py`, which:

- Opens 4 USB cameras (`/dev/video0`, `/dev/video2`, `/dev/video4`,
  `/dev/video6`) via V4L2 with MJPEG at 640×480 / 15 FPS.
- Runs **YOLOv8n** (`yolov8n.pt`) batched across the 4 frames in a single
  GPU call.
- Composites annotated frames into a 2×2 grid window with FPS overlay.

This gave us object **detection** but no notion of distance or per-direction
avoidance decisions. The goal of the new script is to add that.

## 2. Design decisions

Before writing code, three choices were made:

| Choice              | Decision                                  |
| ------------------- | ----------------------------------------- |
| Camera mounting     | Four directions: FRONT / BACK / LEFT / RIGHT (one camera per direction, like DJI) |
| Output              | Visual overlay only (no serial / MQTT yet) |
| Depth estimation    | Monocular: Depth Anything V2 Small        |

**Why monocular depth?** The 4 cameras are single (not stereo pairs), so
true triangulated depth isn't possible. Depth Anything V2 Small is a
modern ViT-based model that produces **relative** depth (larger value =
closer pixel) and runs in real time on a GPU. It's good enough for
"this zone is closer than that one" avoidance decisions; for actual
metric distances you'd need a stereo pair per direction.

## 3. Pipeline overview

For every frame from every camera:

```
        ┌──────────────┐
camera ─┤   capture    │
        └──────┬───────┘
               │ 4 BGR frames
        ┌──────▼────────────────────────────────┐
        │  YOLOv8n  (batched, 4 frames at once) │  → bounding boxes
        │  Depth Anything V2 (batched)          │  → relative depth map
        └──────┬────────────────────────────────┘
               │
        ┌──────▼───────────┐
        │ Per camera:      │
        │  • normalize     │
        │  • split into    │
        │    L/C/R zones   │
        │  • count "close" │
        │    pixels        │
        │  • CLEAR /       │
        │    CAUTION /     │
        │    STOP per zone │
        └──────┬───────────┘
               │
        ┌──────▼───────────┐
        │ Compose 2x2 grid │
        │ + zone bars +    │
        │ direction labels │
        └──────────────────┘
```

## 4. Implementation steps

### Step 1 — Reuse the camera capture and YOLO pieces

The new script keeps the same V4L2 / MJPEG camera setup and the same
batched YOLO call. The two functions `open_camera()` and `placeholder()`
are unchanged from the original.

### Step 2 — Add a direction label per camera

```python
CAMERAS = [0, 2, 4, 6]
CAM_DIRECTIONS = ["FRONT", "RIGHT", "BACK", "LEFT"]
```

You should re-order `CAM_DIRECTIONS` to match the physical mount of each
`/dev/video` index on your rig.

### Step 3 — Load Depth Anything V2 alongside YOLO

```python
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

DEPTH_MODEL = "depth-anything/Depth-Anything-V2-Small-hf"
DEPTH_INPUT = 308   # multiple of 14 (ViT patch size)

proc = AutoImageProcessor.from_pretrained(
    DEPTH_MODEL, size={"height": DEPTH_INPUT, "width": DEPTH_INPUT}
)
depth_model = AutoModelForDepthEstimation.from_pretrained(
    DEPTH_MODEL
).to(torch_device).eval()
```

Two non-obvious choices here:

- **Input size 308 instead of the default 518**: dropped the depth input
  side length from 518 px to 308 px. Must be a multiple of 14 (the ViT
  patch size). This gives roughly a 3× speed-up on the T1000 8GB with
  minimal accuracy loss for proximity decisions.
- **Stay in fp32**: tried `model.half()` (fp16) first, but Depth Anything
  V2 produces `NaN` outputs in fp16 on this GPU. bf16 isn't natively
  accelerated on Turing (T1000), so fp32 is the practical choice.

### Step 4 — Batched depth inference

```python
rgb_imgs = [cv2.cvtColor(f, cv2.COLOR_BGR2RGB) for f in frames_bgr]
with torch.no_grad():
    inputs = proc(images=rgb_imgs, return_tensors="pt")
    pv = inputs["pixel_values"].to(torch_device, dtype=depth_dtype)
    pred = depth_model(pixel_values=pv).predicted_depth   # (4, H', W')
    pred = torch.nn.functional.interpolate(
        pred.unsqueeze(1).float(),
        size=(CAP_H, CAP_W),
        mode="bicubic",
        align_corners=False,
    ).squeeze(1)
depths = pred.cpu().numpy()                               # (4, 480, 640)
```

OpenCV gives us BGR, the HuggingFace processor expects RGB, so we convert
once per frame. The model is fed all 4 frames in one batch, then the
output is bicubically up-sampled back to the camera resolution.

Depth Anything V2's output is **inverse-depth-like**: larger value =
closer pixel.

### Step 5 — Per-frame normalization

```python
dmin, dmax = float(dm.min()), float(dm.max())
dm_n = (dm - dmin) / (dmax - dmin + 1e-6)
```

Because Depth Anything V2 outputs relative depth (no fixed scale), we
normalize each frame to `[0, 1]` per frame. In this normalized space,
values close to `1` correspond to the closest pixels in that frame.

### Step 6 — Zone-based decision logic

Each tile is split into `N_ZONES = 3` equal-width vertical slices
(left / center / right). For each zone we count what fraction of pixels
are "close" (normalized depth above `CLOSE_THRESH = 0.70`):

```python
def zone_statuses(depth_norm):
    h, w = depth_norm.shape
    zone_w = w // N_ZONES
    out = []
    for i in range(N_ZONES):
        x0 = i * zone_w
        x1 = w if i == N_ZONES - 1 else (i + 1) * zone_w
        frac = float((depth_norm[:, x0:x1] > CLOSE_THRESH).mean())
        if   frac >= STOP_FRAC: status = "STOP"     # 0.25 by default
        elif frac >= WARN_FRAC: status = "CAUTION"  # 0.10 by default
        else:                   status = "CLEAR"
        out.append((status, frac))
    return out
```

Thresholds are tunable knobs at the top of the script:

| Knob           | Default | Meaning                                        |
| -------------- | ------- | ---------------------------------------------- |
| `CLOSE_THRESH` | 0.70    | a pixel counts as "close" above this normalized depth |
| `WARN_FRAC`    | 0.10    | ≥10 % close pixels in a zone → **CAUTION**    |
| `STOP_FRAC`    | 0.25    | ≥25 % close pixels in a zone → **STOP**       |
| `N_ZONES`      | 3       | left / center / right — bump to 5 for finer zones |

### Step 7 — Per-tile visualization

For each tile we:

1. Take the YOLO-annotated frame (`res.plot()`).
2. Build a colored heatmap from the normalized depth
   (`cv2.applyColorMap(..., cv2.COLORMAP_INFERNO)`).
3. Alpha-blend the two (`addWeighted(annotated, 0.6, heat, 0.4, 0)`).
4. Draw three colored zone bars across the top — green (CLEAR),
   yellow (CAUTION), red (STOP) — with the percentage of "close" pixels.
5. Draw the direction label (FRONT / BACK / LEFT / RIGHT) at the bottom.

The 4 tiles are then stacked into a 2×2 grid with an FPS overlay, same
as the original script.

## 5. Dependencies that were installed

The original detect script already depended on `ultralytics`, `opencv-python`,
`numpy`, `torch`. Two more were needed:

```bash
pip install transformers pillow
```

A subtle gotcha: the system-installed Pillow (9.0.1) was too old for the
current `transformers` (`PIL.Image.Resampling` only exists in Pillow ≥ 9.1).
Fixed with:

```bash
pip install --user --upgrade "pillow>=10.0"
```

## 6. Performance on this machine

GPU: **NVIDIA T1000 8GB** (Turing, compute 7.5).

| Config                              | FPS (4 cameras, end to end) |
| ----------------------------------- | --------------------------- |
| Depth input 518 (default), fp32     | ~1.9 FPS                    |
| Depth input 308, fp32 (**default**) | ~6.4 FPS                    |
| Depth input 252, fp32               | ~9 FPS (rough)              |
| Depth input 196, fp32               | ~12 FPS (rough)             |

Capture is capped at 15 FPS. Depth inference dominates wall time. If you
need more headroom, lower `DEPTH_INPUT` (always keep it a multiple of 14).

## 7. Running the system

```bash
cd /home/trlspace/obstacle_dection_four_cameras
python3 obstacle_avoidance_four_cameras.py
```

The window is shown on `DISPLAY=:1`. Press **q** or **Esc** to quit.

First run downloads ~100 MB of depth-model weights from HuggingFace; later
runs use the local cache.

## 8. Known limitations

- **Relative depth, not metric.** You cannot read meters off the screen.
  "STOP" means *something is close relative to the rest of this view*,
  not "obstacle at 1.2 m". For real distances, swap a single camera for
  a stereo pair on that direction and use `cv2.StereoSGBM` — the rest
  of the pipeline stays the same.
- **Sky / bright backgrounds.** Depth Anything V2 sometimes labels very
  bright sky regions as "close". If you see false STOPs in the top of a
  zone outdoors, mask out the upper N rows before running `zone_statuses`.
- **Camera mount mapping.** `CAM_DIRECTIONS` is just a list — you have
  to physically verify that index 0 really is FRONT, etc.
- **Visualization only.** The script does not emit any control signals.
  Hook into the `statuses` list (per camera, per zone) to publish over
  serial / UDP / MQTT when you're ready to drive a real vehicle.

## 9. Files in this directory

```
detect_four_cameras.py              original YOLO-only script (unchanged)
obstacle_avoidance_four_cameras.py  NEW: YOLO + Depth Anything V2 + zones
yolov8n.pt                          YOLO weights
obstacle_avoidance.md               this document
```
