# Torque Vectoring UI (Linux)

Minimal steps to run the UI on a Linux machine with a USB-to-CAN adapter.

## Setup
```bash
# 1) Create and activate a virtual env
python3 -m venv .venv
source .venv/bin/activate

# 2) Install Python dependencies
pip install -r requirements.txt
```

## Bring up CAN (SocketCAN can0)
```bash
# Requires sudo; adjusts to the last ttyACM* seen by dmesg
sudo bash setup_can_usb.sh
```
If you prefer manual setup, ensure `can0` exists and is up (e.g., `sudo ip link set can0 up type can bitrate 1000000`).

## Run the UI
```bash
source .venv/bin/activate
python torque_vectoring_ui.py
```

## Notes
- DBC sending is enabled when a DBC file is loaded; `cantools` is already in requirements.
- If no CAN hardware is connected, the app will show "CAN Disconnected" and skip sending; you can still explore the UI.
- Adjust the CAN channel/interface in `torque_vectoring_ui.py` if your setup differs (e.g., non-`can0` name).

ls /dev/ttyACM*

ls /dev/ttyACM*
sudo modprobe can && sudo modprobe can_raw && sudo slcand -o -c -s8 /dev/ttyACM0 can0 && sudo ip link set can0 up && sudo ip link set can0 txqueuelen 1000