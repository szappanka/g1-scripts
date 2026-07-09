# G1 Scripts

Personal scripts for the Unitree G1 robot.

## Dependencies

`unitree_sdk2py` is not on PyPI — install it straight from GitHub (its pinned
dependency `cyclonedds==0.10.2` only ships Windows wheels for Python 3.7–3.10,
so use one of those versions on Windows or the pip install will try to
compile cyclonedds from source and fail):

```bash
pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
pip install -r requirements.txt
```

## Connection

See [MAC_CONNECTION.md](MAC_CONNECTION.md) for how to connect a Mac to the G1 over Ethernet (includes SSH over WiFi via hostname), or [WINDOWS_CONNECTION.md](WINDOWS_CONNECTION.md) for Windows.

- Mac interface: `en8`, IP: `192.168.123.222`
- Windows: find your adapter with `Get-NetAdapter`, IP: `192.168.123.222`
- Robot IP: `192.168.123.164`
- SSH over university WiFi: `ssh unitree@ubuntu.local` (hostname-based, survives DHCP IP changes)

## Scripts

### Audio

- `example/g1/audio/g1_audio_led_demo.py` — TTS + rainbow LED animation (robot doesn't move)
- `example/g1/audio/g1_chat_gemini.py` — typed chat → Gemini → robot speaks
- `example/g1/audio/g1_tts_voice_test.py` — TTS voice tester

### Lidar

- `example/g1/lidar/g1_lidar_viewer.py` — lidar point cloud viewer

### Movement

- `example/g1/high_level/g1_handshake_grab_demo.py` — handshake with grab detection via arm_sdk
- `example/g1/high_level/g1_pose_mimic_preview.py` — webcam + MediaPipe Pose overlay, no robot (test the vision pipeline first)
- `example/g1/high_level/g1_pose_mimic.py` — robot arm(s) + waist follow your upper-body movement live via arm_sdk (`--dry-run --show` to test without connecting; legs/balance always stay on the robot's built-in controller, not a whole-body/gait mimic)

## Usage

```bash
# Gemini voice assistant (set API key first)
export GEMINI_API_KEY=your-key-here
python3 example/g1/audio/g1_chat_gemini.py en8

# Audio + LED demo
python3 example/g1/audio/g1_audio_led_demo.py en8
```
