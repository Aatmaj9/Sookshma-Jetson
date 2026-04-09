#!/bin/bash
docker compose -f .devcontainer/docker-compose.yml up -d
docker exec -it masv01 bash
