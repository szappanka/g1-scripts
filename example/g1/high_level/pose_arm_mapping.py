"""
Felsőtest-póz becslés MediaPipe Pose 3D "world landmark"-okból -- robot-független
geometria, amit a pose preview és a tényleges G1 mimic szkript is használ.

Bemenet: egy MediaPipe `pose_world_landmarks.landmark` lista (33 elem, méterben,
csípő-középpont körüli, .x/.y/.z attribútumokkal). Nem importál mediapipe-ot,
csak indexeli a listát -- így a preview/robot szkriptektől függetlenül tesztelhető.

Kimenet EMBER-relatív, robot-független szögek radiánban:
  "left"/"right" (kar, a törzshöz képest):
    pitch       -- 0 = kar lóg lefelé (nyugalmi), + = előre/felfelé emelve
    roll        -- 0 = kar a törzs mellett, + = oldalra/kifelé emelve (a testtől távolodva)
    elbow_bend  -- 0 = nyújtott kar, + = egyre jobban behajlítva (kb. pi/2..2 a tipikus tartomány)
  "torso" (derék, egy FELTÉTELEZETT világ-vertikálishoz képest -- ld. lent):
    pitch -- 0 = egyenesen áll, + = előre hajol
    roll  -- 0 = egyenesen áll, + = jobbra dől
    yaw   -- 0 = szemben a kamerával, + = jobbra fordul (váll-vonal elfordulása)

FONTOS -- két külön bizonytalansági forrás:
1) A kar pitch/roll csak egy egyszerűsített, egy-egy skalárra bontott közelítés
   (nem teljes 3-DOF váll-orientáció).
2) A torso szögek egy FELTÉTELEZETT világ-tengelyt használnak (Y lefelé, Z a
   kamera felé) -- ez csak akkor helyes, ha a kamera nagyjából szintben áll és
   szemből látja a felhasználót. Ferde/döntött kamera esetén a torso pitch/roll
   nullpontja el fog csúszni. Ellenőrizd --dry-run --show kapcsolóval, mielőtt
   hardveren használnád.

A robot-specifikus előjeleket/skálázást NEM ez a modul dönti el, azt a hívó
(g1_pose_mimic.py) végzi, mert az a konkrét robot kalibrációjától függ.
"""

import math

import numpy as np

LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_ELBOW, RIGHT_ELBOW = 13, 14
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24

MIN_VISIBILITY = 0.5
MIN_SEGMENT_LENGTH = 0.05  # meter -- ennel rovidebb "felkar"/"alkar" fizikailag lehetetlen,
# tehat depth-becslesi hiba jele (leggyakoribb MediaPipe hibaforras: a Z tengely) -- ilyenkor
# inkabb kihagyjuk a keretet, mintsem egy elfajult (pl. konyok=pi) szoget szamoljunk belole.

# Feltételezett, kamerához rögzített világ-tengelyek (nem a testből származtatva) --
# csak a torso dőlés/elfordulás méréséhez kell, ahhoz képest, hogy a kamera "egyenes".
WORLD_UP = np.array([0.0, -1.0, 0.0])
WORLD_FORWARD = np.array([0.0, 0.0, -1.0])
WORLD_RIGHT = np.array([1.0, 0.0, 0.0])


def _p(lm):
    return np.array([lm.x, lm.y, lm.z], dtype=float)


def _normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else v


def _angle_between(v1, v2):
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return math.acos(cos_a)


def _visibility(lm):
    return getattr(lm, "visibility", 1.0)


def _fading_atan2(y, x, eps=0.03):
    """atan2(y,x), de 0-hoz tartva, ha (y,x) nagysaga elhanyagolhato -- elkeruli, hogy
    egy majdnem-0/0 pontban (fizikai szingularitas, pl. tiszta oldalra-emeles/dolés)
    a nyers atan2 zajra erzekeny, akar pi-ig ugro erteket adjon."""
    mag = math.hypot(y, x)
    return math.atan2(y, x) * (mag / (mag + eps))


def _arm_angles(shoulder_p, elbow_p, wrist_p, forward, up, right, is_right_side):
    upper = elbow_p - shoulder_p
    lower = wrist_p - elbow_p

    f = float(np.dot(upper, forward))
    u = float(np.dot(upper, up))
    r = float(np.dot(upper, right))

    pitch = _fading_atan2(f, -u)
    lateral = r if is_right_side else -r
    roll = math.atan2(lateral, math.hypot(f, -u))

    straight_angle = _angle_between(-upper, lower)  # pi = nyújtott, 0 = teljesen behajlítva
    elbow_bend = math.pi - straight_angle

    return pitch, roll, elbow_bend


def _torso_angles(mid_shoulder, mid_hip, left_shoulder, right_shoulder):
    torso = mid_shoulder - mid_hip  # a gerinc iranya, csipotol vall fele -- egyenes
    # allasnal ez NAGYJABOL EGYEZIK a WORLD_UP iranyaval (szemben a kar "up"
    # referenciajaval, ahol a nyugalmi kar/felkar EPP ELLENTETES iranyu -- ezert
    # itt +u kell a nevezobe, ott -u).

    f = float(np.dot(torso, WORLD_FORWARD))
    u = float(np.dot(torso, WORLD_UP))
    r = float(np.dot(torso, WORLD_RIGHT))

    pitch = _fading_atan2(f, u)   # elore hajlas
    roll = _fading_atan2(r, u)    # oldalra doles

    # Empirikusan igazolva (vezetett teszt, felhasznalo ellenorizve szemben allt a
    # kameraval): a nyers (right_shoulder - left_shoulder) iranyu vallvonallal a
    # yaw kb. +pi-t adott 0 helyett -- tehat ez a vektor a vart iranyhoz kepest
    # forditva all. Innen a negalt sorrend (left - right).
    shoulder_line = left_shoulder - right_shoulder
    yaw = math.atan2(float(np.dot(shoulder_line, WORLD_FORWARD)), float(np.dot(shoulder_line, WORLD_RIGHT)))

    return {"pitch": pitch, "roll": roll, "yaw": yaw}


def compute_pose_targets(landmarks):
    """Visszaad egy dict-et: {"left": {...}, "right": {...}, "torso": {...}},
    ahol a kar mezoi None-ok, ha az adott kar landmarkjai nem elég láthatóak, és
    "torso" is None, ha a vall/csipo landmarkok nem elég láthatóak."""
    ls, rs = landmarks[LEFT_SHOULDER], landmarks[RIGHT_SHOULDER]
    lh, rh = landmarks[LEFT_HIP], landmarks[RIGHT_HIP]
    le, re = landmarks[LEFT_ELBOW], landmarks[RIGHT_ELBOW]
    lw, rw = landmarks[LEFT_WRIST], landmarks[RIGHT_WRIST]

    mid_sh = (_p(ls) + _p(rs)) / 2.0
    mid_hip = (_p(lh) + _p(rh)) / 2.0

    up = _normalize(mid_sh - mid_hip)
    right = _normalize(_p(rs) - _p(ls))
    forward = _normalize(np.cross(right, up))

    result = {}
    for side, shoulder, elbow, wrist, is_right in (
        ("left", ls, le, lw, False),
        ("right", rs, re, rw, True),
    ):
        visible = min(_visibility(shoulder), _visibility(elbow), _visibility(wrist)) >= MIN_VISIBILITY
        upper_len = np.linalg.norm(_p(elbow) - _p(shoulder))
        lower_len = np.linalg.norm(_p(wrist) - _p(elbow))
        plausible = upper_len >= MIN_SEGMENT_LENGTH and lower_len >= MIN_SEGMENT_LENGTH
        if not (visible and plausible):
            result[side] = None
            continue
        pitch, roll, elbow_bend = _arm_angles(_p(shoulder), _p(elbow), _p(wrist), forward, up, right, is_right)
        result[side] = {"pitch": pitch, "roll": roll, "elbow_bend": elbow_bend}

    torso_visible = min(_visibility(ls), _visibility(rs), _visibility(lh), _visibility(rh)) >= MIN_VISIBILITY
    result["torso"] = _torso_angles(mid_sh, mid_hip, _p(ls), _p(rs)) if torso_visible else None

    return result
