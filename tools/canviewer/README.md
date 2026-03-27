# Windows CANViewer

FSAE CAN Viewer — Modern Dashboard for CANable 2 (gs_usb)
Parses `mkvii.dbc` and displays live decoded CAN signals in a dark-themed, tabbed dashboard GUI.

## Setup Instructions

### 1. Python & Dependencies Setup

1. Make sure you have **Python 3.x** installed.
2. Open a terminal (Command Prompt or PowerShell) and navigate to this repository folder.
3. Install the required Python packages using `pip`:
   ```bash
   pip install -r requirements.txt
   ```

### 2. WinUSB Driver Installation (Zadig)

To communicate with the CANable (gs_usb firmware) natively on Windows, you must install the **WinUSB** driver for the device. Without this, Python cannot interface with the hardware via USB.

1. **Plug in** your CANable 2 device to your computer via USB.
2. Download **Zadig** from [https://zadig.akeo.ie/](https://zadig.akeo.ie/).
3. Open Zadig from your Downloads folder.
4. Go to **Options** -> **List All Devices** in the top menu.
5. In the large dropdown box, select your CANable device. It may be named something like `CANable`, `gs_usb fall-back`, `candleLight USB to CAN adapter`, etc.
6. Verify the current driver (left box) is whatever Windows picked, and that the target driver (right box with the green arrow) is set to **WinUSB**.
7. Click the **Install Driver** (or **Replace Driver**) button and wait for it to complete.

### 3. Running the Dashboard

Once the dependencies are installed and the WinUSB driver is active, you can launch the application:

1. Guarantee your DBC file (`mkvii.dbc`) is placed in the same directory as the script.
2. Run the viewer:
   ```bash
   python canviewer.py
   ```
3. In the viewer window:
   - Click the **Refresh** button in the top right to populate the Dropdown Device list.
   - Choose your `gs_usb` device.
   - Click **Connect** to start receiving CAN telemetry.
