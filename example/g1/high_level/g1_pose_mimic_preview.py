"""
Webkamera + MediaPipe Pose élő előnézet -- ROBOT NÉLKÜL. Csak megmutatja a
vázfelismerést és a pose_arm_mapping.py által számolt kar- és torso-szögeket,
hogy a kamera+geometria lánc helyesen működik-e, mielőtt bármi elmenne a robotnak.

Telepítés:
    pip install mediapipe

Használat:
    python3 g1_pose_mimic_preview.py
    python3 g1_pose_mimic_preview.py --camera 1

Kilépés: 'q' a videóablakban, vagy Ctrl+C.
"""

import argparse

import cv2
import mediapipe as mp

from pose_arm_mapping import compute_pose_targets


def main():
    parser = argparse.ArgumentParser(description="MediaPipe Pose előnézet -- robot nélkül")
    parser.add_argument("--camera", type=int, default=0, help="kamera index (alap: 0)")
    args = parser.parse_args()

    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Nem sikerült megnyitni a(z) {args.camera} kamerát.")

    with mp_pose.Pose(model_complexity=1, min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)

            if results.pose_landmarks:
                mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            if results.pose_world_landmarks:
                targets = compute_pose_targets(results.pose_world_landmarks.landmark)
                y = 30
                for side in ("left", "right"):
                    t = targets[side]
                    if t is None:
                        text = f"{side}: nem lathato"
                    else:
                        text = (f"{side}: pitch={t['pitch']:+.2f} roll={t['roll']:+.2f} "
                                f"elbow_bend={t['elbow_bend']:+.2f}")
                    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    y += 25
                torso = targets["torso"]
                if torso is None:
                    text = "torso: nem lathato"
                else:
                    text = f"torso: pitch={torso['pitch']:+.2f} roll={torso['roll']:+.2f} yaw={torso['yaw']:+.2f}"
                cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            cv2.imshow("G1 pose mimic preview (q = kilepes)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
