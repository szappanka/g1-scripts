# G1 Scripts

Personal scripts for the Unitree G1 robot.

## Dependencies

```bash
pip install unitree_sdk2py google-genai
```

## Connection

See [MAC_CONNECTION.md](MAC_CONNECTION.md) for how to connect a Mac to the G1 over Ethernet.

- Mac interface: `en8`, IP: `192.168.123.222`
- Robot IP: `192.168.123.164`

## Scripts

### Audio

- `example/g1/audio/g1_audio_led_demo.py` — TTS + rainbow LED animation (robot doesn't move)
- `example/g1/audio/g1_chat_gemini.py` — typed chat → Gemini → robot speaks
- `example/g1/audio/g1_tts_voice_test.py` — TTS voice tester

### Lidar

- `example/g1/lidar/g1_lidar_viewer.py` — lidar point cloud viewer

## Usage

```bash
# Gemini voice assistant (set API key first)
export GEMINI_API_KEY=your-key-here
python3 example/g1/audio/g1_chat_gemini.py en8

# Audio + LED demo
python3 example/g1/audio/g1_audio_led_demo.py en8
```
