# Reference: https://docs.docker.com/reference/dockerfile/
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python-is-python3 \
    python3 \
    python3-pip \
    python3-venv

COPY requirements.txt /tmp/requirements.txt
RUN python -m venv /venv
RUN /venv/bin/pip install -r /tmp/requirements.txt
