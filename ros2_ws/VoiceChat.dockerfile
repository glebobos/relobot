FROM python:3.10-slim

# Install system audio utilities (PulseAudio & ALSA), curl, tmux, git, and build essentials
RUN apt-get update && apt-get install -y --no-install-recommends \
    pulseaudio-utils \
    alsa-utils \
    libasound2 \
    libgl1 \
    libglib2.0-0 \
    curl \
    ca-certificates \
    build-essential \
    tmux \
    git \
    procps \
    && rm -rf /var/lib/apt/lists/*

# Install multi-arch static ttyd binary (v1.7.7)
RUN ARCH=$(uname -m) && \
    curl -sLo /usr/local/bin/ttyd "https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.${ARCH}" && \
    chmod +x /usr/local/bin/ttyd

# Copy requirements and install via pip
COPY src/voice_chat_server/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Install uv package runner
RUN pip install --no-cache-dir uv

# Download Piper Optimus Prime voice model using model_manager.py as Single Source of Truth
ENV PIPER_MODELS_DIR=/opt/piper_models
COPY src/voice_chat_server/model_manager.py /tmp/model_manager.py
RUN python3 /tmp/model_manager.py /opt/piper_models && rm /tmp/model_manager.py

# Workspace directory
WORKDIR /ros2_ws

# Server startup script
RUN echo '#!/bin/bash\n\
set -e\n\
echo "[VoiceChat] Starting ReloBot AI Voice Chat & AGY Terminal Server..."\n\
exec python3 /ros2_ws/src/voice_chat_server/server.py' > /start_voice_chat.sh && \
chmod +x /start_voice_chat.sh

EXPOSE 8765 7681

CMD ["/start_voice_chat.sh"]
