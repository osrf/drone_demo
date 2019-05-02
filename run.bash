#!/usr/bin/env bash


IMG=tfoote/drone_demo

ARGS=("$@")

rocker --x11 --nvidia --user --home --pulse  $IMG ${ARGS[@]}


