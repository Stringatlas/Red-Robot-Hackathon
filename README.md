# RED ROBOTTTT

Follow [these steps](https://docs.google.com/document/d/1g0XnY5zaHTjELDTqYydpKX9lNMo9_3tNZBG45k9CsoQ/edit?usp=sharing) to get started with the robot code!

## VS Code editing with Arduino builds

1. Install the VS Code **C/C++** extension (`ms-vscode.cpptools`). Install Arduino CLI if you do not already have it.
2. In Arduino IDE's Boards Manager, install **esp32 by Espressif Systems**. In Library Manager, install **NimBLE-Arduino**. This project was checked with ESP32 core 3.3.12 and NimBLE-Arduino 2.5.2.
3. Open this repository folder in VS Code. Run **Tasks: Run Task** → **Arduino: Configure IntelliSense** from the Command Palette. Run it again after changing boards or updating the core or library.
4. Edit in VS Code and build/upload with Arduino IDE as usual. You can also use **Terminal: Run Build Task** to compile with Arduino CLI.

The included VS Code task targets `esp32:esp32:esp32s3` (**ESP32S3 Dev Module**), matching the ESP32-S3 pins used by the sketch. If your robot uses a different board configuration, select the right board in Arduino IDE and update the FQBN in `.vscode/tasks.json`; pass that same FQBN to `python3 scripts/configure_vscode.py --fqbn <your-fqbn>`.

The setup task generates local IntelliSense settings and a compilation database from the installed Arduino core. Those generated files are ignored by Git. `Arduino.h`, `SPI.h`, `driver/mcpwm.h`, and `freertos/semphr.h` come from the ESP32 core and its SDK; they are not project files.
