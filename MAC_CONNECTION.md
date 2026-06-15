# G1 csatlakozás Mac-en

## 1. Interfész neve (ha nem tudod)

```bash
networksetup -listallhardwareports | grep -A1 "USB" | grep Device | awk '{print $2}'
```

Jelenleg: `en6`

## 2. IP beállítás (minden csatlakozáskor kell)

```bash
sudo ifconfig en6 192.168.123.222 netmask 255.255.255.0 up
```

## 3. Ellenőrzés

```bash
ping 192.168.123.164
```

## 4. Script futtatás

```bash
python3 example/g1/audio/g1_chat_gemini.py en6
```

---

Mac IP: `192.168.123.222` | Robot IP: `192.168.123.164`
