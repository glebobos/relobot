FROM python:3.10-slim

# Install system audio utilities (PulseAudio & ALSA), curl, and build essentials
RUN apt-get update && apt-get install -y --no-install-recommends \
    pulseaudio-utils \
    alsa-utils \
    libasound2 \
    curl \
    ca-certificates \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install via pip
COPY src/voice_chat_server/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Download Piper Optimus Prime voice model using the download script as Single Source of Truth
ENV PIPER_MODELS_DIR=/opt/piper_models
COPY src/voice_chat_server/download_models.py /tmp/download_models.py
RUN python3 /tmp/download_models.py /opt/piper_models && rm /tmp/download_models.py

# Workspace directory
WORKDIR /ros2_ws

# Server startup script
RUN echo '#!/bin/bash\n\
set -e\n\
echo "[VoiceChat] Starting ReloBot AI Voice Chat Server..."\n\
exec python3 /ros2_ws/src/voice_chat_server/server.py' > /start_voice_chat.sh && \
chmod +x /start_voice_chat.sh

EXPOSE 8765

CMD ["/start_voice_chat.sh"]
