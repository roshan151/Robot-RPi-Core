#!/usr/bin/env python3
"""Manual test for the vision microservice. Press q to quit any window.

  python vision_test.py detect   # live stream with boxes for every detected object
  python vision_test.py enroll   # 60 s face capture (one person only), then asks for a name
  python vision_test.py match    # live face recognition against the enrolled gallery
"""
import argparse, os, time
import cv2, requests

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("mode", choices=["detect", "enroll", "match"])
p.add_argument("--url", default=os.getenv("VISION_SERVICE_BASE_URL", "http://127.0.0.1:8080"))
p.add_argument("--camera", type=int, default=0)
p.add_argument("--seconds", type=float, default=60, help="enroll capture length")
a = p.parse_args()
a.url = a.url.rstrip("/")
print(requests.get(a.url + "/healthz", timeout=5).json())
GREEN, RED, YELLOW = (0, 255, 0), (0, 0, 255), (0, 255, 255)


def jpg(frame):
    return cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


def post(path, frame, **data):  # None on 404 (no face) or busy
    r = requests.post(a.url + path, files={"image": ("f.jpg", jpg(frame), "image/jpeg")}, data=data, timeout=5)
    return r.json() if r.ok else None


def label(frame, text, xy=(10, 30), color=YELLOW, bbox=None):
    if bbox:
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        xy = (x1, max(y1 - 6, 14))
    cv2.putText(frame, text, xy, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def frames():  # yields (seq, frame) until q is pressed
    cap, seq = cv2.VideoCapture(a.camera), 0
    while cv2.waitKey(1) & 0xFF != ord("q"):
        ok, f = cap.read()
        if not ok:
            raise SystemExit(f"cannot read camera {a.camera}")
        yield (seq := seq + 1), f


if a.mode == "detect":
    for seq, f in frames():
        for d in (post("/v1/detect", f, seq=seq) or {}).get("detections", []):
            label(f, f"{d['class']} {d['confidence']:.2f}", color=GREEN, bbox=d["bbox"])
        cv2.imshow("detect", f)

elif a.mode == "match":
    for seq, f in frames():
        if face := post("/v1/faces/embed", f, seq=seq):
            m = requests.post(a.url + "/v1/faces/match", json={"embedding": face["embedding"]}, timeout=5).json()
            best = m["matches"][0] if m["matches"] else None
            hit = best and best["similarity"] >= m["threshold"]
            label(f, f"{best['label']} {best['similarity']:.2f}" if hit else "unknown",
                  color=GREEN if hit else RED, bbox=face["face_bbox"])
        cv2.imshow("match", f)

else:  # enroll: keep up to 16 good single-face frames (the service's per-call limit), evenly spaced
    shots, t0, last = [], time.time(), 0.0
    for seq, f in frames():
        if (left := a.seconds - (time.time() - t0)) <= 0:
            break
        clean, face = f.copy(), post("/v1/faces/embed", f, seq=seq)
        if face and face["face_count"] > 1:
            raise SystemExit(f"Rejected: {face['face_count']} faces in view. Capture one person at a time.")
        if face:
            label(f, f"quality {face['quality']:.2f}", color=GREEN, bbox=face["face_bbox"])
            if face["quality"] >= 0.6 and time.time() - last >= a.seconds / 16:
                shots.append(clean)
                last = time.time()
        label(f, f"{left:4.1f}s left   captured {len(shots)}/16")
        cv2.imshow("enroll", f)
    cv2.destroyAllWindows()
    if not shots:
        raise SystemExit("No usable face captured; nothing saved.")
    name = input(f"Captured {len(shots)} images. Name for this person: ").strip()
    r = requests.post(a.url + "/v1/faces/enroll", data={"label": name}, timeout=60,
                      files=[("images", (f"{i}.jpg", jpg(s), "image/jpeg")) for i, s in enumerate(shots)])
    print("Saved:" if r.ok else f"Enroll failed ({r.status_code}):", r.json())
