#!/bin/bash
docker-compose -f simulation-docker-compose.yml build && docker-compose -f simulation-docker-compose.yml run gazebo bash
