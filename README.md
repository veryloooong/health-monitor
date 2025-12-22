# Health monitor

## Prerequisites

- Arduino CLI
- Docker/Podman

## Build instructions

For the board:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 health-monitor

arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 health-monitor
```

For the MQTT server and logger:

```sh
cd server && docker compose up --build -d 
```

Then access the MQTT server at `mqtt://localhost:1883` and view the logger CSV at `server/logger/health_data.csv`.

For the web client: Just open the file in the browser
