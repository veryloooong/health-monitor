# Health monitor

## Prerequisites

- Arduino CLI
- Docker/Podman

## Build instructions

For the board:

- Upload the sketch to the board with Arduino CLI or other tools idk

For the MQTT server:

```sh
docker compose -p server/ up
```

For the web client: Just open the file in the browser
