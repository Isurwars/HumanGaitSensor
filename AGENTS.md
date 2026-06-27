# PlatformIO Agent Guardrails

## Core Stack
- **Framework:** PlatformIO Core (CLI-driven)
- **Target Platform:** ESP32 / Arduino / ESP-IDF (as defined in platformio.ini)

## Agent Execution Rules
1. **Compilation Commands:** Do not use `gcc`, `clang`, or generic `make`. All code verification, builds, and compilation checks must be executed via the PlatformIO CLI in the terminal using:
   `pio run`
2. **Uploading & Monitoring:** When asked to upload code or check serial outputs, run:
   `pio run --target upload` or `pio device monitor`
3. **Library Management:** Do not manually download source files for external libraries. Use PlatformIO's package manager by adding declarations to `platformio.ini` or letting the agent run:
   `pio pkg install --library <lib_name>`
4. **Environment Awareness:** Always inspect the local `platformio.ini` file before suggesting architectural code changes to ensure compatibility with the exact board variant configured.