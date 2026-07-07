# G1 csatlakozás Windows-on

## 1. Interfész neve (ha nem tudod)

PowerShell-ben (admin):

```powershell
Get-NetAdapter | Select-Object Name, InterfaceDescription, Status
```

Keresd azt, amelyik a robothoz kötött USB-Ethernet adapter (pl. `Ethernet 2`).

## 2. IP beállítás (minden csatlakozáskor kell)

```powershell
netsh interface ip set address name="Ethernet 2" static 192.168.123.222 255.255.255.0
```

(Cseréld ki az `"Ethernet 2"`-t a saját adapter nevére.)

## 3. Ellenőrzés

```powershell
ping 192.168.123.164
```

## 4. Script futtatás

```powershell
python example/g1/audio/g1_chat_gemini.py "Ethernet 2"
```

(Az interfész paramétert ugyanúgy add meg, mint amit a `Get-NetAdapter` mutatott.)

---

Windows IP: `192.168.123.222` | Robot IP: `192.168.123.164`
