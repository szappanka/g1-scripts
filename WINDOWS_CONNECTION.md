# G1 csatlakozás Windows-on

## 0. Python csomagok telepítése

A `unitree_sdk2py` nincs fent a PyPI-n, GitHub-ról kell telepíteni. A pinnelt
`cyclonedds==0.10.2` függőségnek **csak Python 3.7–3.10-hez van Windows
wheel-je** — ha 3.11+ Python fut a gépen, a pip forrásból próbálja buildelni
a cyclonedds-t (kell hozzá Visual Studio Build Tools + CMake), ami tipikusan
elhasal. Legegyszerűbb megoldás: Python 3.10 egy külön venv-ben.

```powershell
py -3.10 -m venv .venv
.venv\Scripts\activate
pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
pip install -r requirements.txt
```

(Ha nincs telepítve a 3.10-es Python, töltsd le a python.org-ról — a "py
launcher" komponenst is pipáld be telepítéskor.)

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
