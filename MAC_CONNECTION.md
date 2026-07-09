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

## SSH az egyetemi WiFi-n keresztül (hostname alapú)

Az egyetemi WiFi DHCP-vel osztja ki a robot IP-jét, ez naponta változhat, így
IP cím alapján nem érdemes rákötni. Megoldás: a robot hostname-je (`ubuntu`)
mDNS-en keresztül hirdetve van a hálózaton (az `avahi-daemon` szolgáltatás
felel érte), így a hostname-mel lehet rákötni IP cím helyett — ez minden nap
ugyanaz marad, függetlenül attól, hogy a router aznap milyen IP-t oszt ki.

```bash
ssh unitree@ubuntu.local
```

Fontos: ezt a Mac termináljában kell futtatni, nem a robot termináljában —
ott a robot saját magára próbálna csatlakozni. Előfordulhat, hogy IPv6 címet
ad vissza feloldáskor, ez nem számít, nem kell vele foglalkozni.
