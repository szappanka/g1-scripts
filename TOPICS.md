# G1 DDS Topics & Services Reference

G1 (és H1-2) robothoz tartozó topic/service nevek. Az IDL üzenettípusok a `unitree_hg` névtérből valók.

## DDS Topics

| Topic | Irány | Típus | Leírás |
|---|---|---|---|
| `rt/lowcmd` | → robot | `unitree_hg.msg.dds_.LowCmd_` | Low-level motor parancsok |
| `rt/lowstate` | ← robot | `unitree_hg.msg.dds_.LowState_` | Low-level állapot (motor, IMU, BMS) |
| `rt/arm_sdk` | → robot | `unitree_hg.msg.dds_.LowCmd_` | Kar közvetlen SDK vezérlés |
| `rt/utlidar/cloud_livox_mid360` | ← robot | — | Lidar nyers scan, ~10 Hz |
| `rt/unitree/slam_mapping/points` | ← robot | — | SLAM akkumulált térkép |

## Services

| Service neve | Konstans | Leírás |
|---|---|---|
| `voice` | `AUDIO_SERVICE_NAME` | TTS, ASR, hangerő, LED |
| `sport` | `LOCO_SERVICE_NAME` | Mozgás, egyensúly, sebesség |
| `arm` | `ARM_ACTION_SERVICE_NAME` | Kar akció végrehajtás |

## Audio API ID-k (service: `voice`)

| Konstans | ID | Leírás |
|---|---|---|
| `ROBOT_API_ID_AUDIO_TTS` | 1001 | Szöveg → hang |
| `ROBOT_API_ID_AUDIO_ASR` | 1002 | Hang → szöveg |
| `ROBOT_API_ID_AUDIO_START_PLAY` | 1003 | WAV lejátszás indítás |
| `ROBOT_API_ID_AUDIO_STOP_PLAY` | 1004 | WAV lejátszás leállítás |
| `ROBOT_API_ID_AUDIO_GET_VOLUME` | 1005 | Hangerő lekérdezés |
| `ROBOT_API_ID_AUDIO_SET_VOLUME` | 1006 | Hangerő beállítás |
| `ROBOT_API_ID_AUDIO_SET_RGB_LED` | 1010 | Fej LED szín beállítás |

## Loco API ID-k (service: `sport`)

| Konstans | ID | Leírás |
|---|---|---|
| `ROBOT_API_ID_LOCO_GET_FSM_ID` | 7001 | FSM állapot lekérdezés |
| `ROBOT_API_ID_LOCO_GET_BALANCE_MODE` | 7003 | Egyensúly mód lekérdezés |
| `ROBOT_API_ID_LOCO_GET_SWING_HEIGHT` | 7004 | Lépés magasság lekérdezés |
| `ROBOT_API_ID_LOCO_GET_STAND_HEIGHT` | 7005 | Állás magasság lekérdezés |
| `ROBOT_API_ID_LOCO_SET_FSM_ID` | 7101 | FSM állapot váltás |
| `ROBOT_API_ID_LOCO_SET_BALANCE_MODE` | 7102 | Egyensúly mód beállítás |
| `ROBOT_API_ID_LOCO_SET_SWING_HEIGHT` | 7103 | Lépés magasság beállítás |
| `ROBOT_API_ID_LOCO_SET_STAND_HEIGHT` | 7104 | Állás magasság beállítás |
| `ROBOT_API_ID_LOCO_SET_VELOCITY` | 7105 | Sebesség beállítás |

## Arm API ID-k (service: `arm`)

| Konstans | ID | Leírás |
|---|---|---|
| `ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION` | 7106 | Akció végrehajtás |
| `ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST` | 7107 | Elérhető akciók listája |

## IDL üzenettípusok (unitree_hg — G1/H1-2)

```python
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_,         # motor parancs küldés
    LowState_,       # teljes robot állapot
    MotorCmd_,       # egy motor parancsa
    MotorState_,     # egy motor állapota
    HandCmd_,        # kéz parancs
    HandState_,      # kéz állapot
    IMUState_,       # IMU (gyro, acc, orientáció)
    BmsState_,       # akkumulátor állapot
    BmsCmd_,         # akkumulátor parancs
    PressSensorState_,  # nyomásérzékelő
    MainBoardState_,    # főlap állapot
)
```

> **Megjegyzés:** Go2/B2/H1/B2w/Go2w robotokhoz `unitree_go` IDL kell, G1/H1-2-höz `unitree_hg`.
