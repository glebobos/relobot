/**
 * ReloBot AI Chat & WebSocket Service
 * Handles bidirectional WebSocket communication with the Voice Chat Server,
 * real-time token streaming, and browser-side Web Audio PCM playback.
 */

export class ChatService {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.isReconnecting = false;
        this.reconnectTimer = null;

        // Web Audio PCM playback
        this.audioContext = null;
        this.activeAudioSources = [];
        this.nextPlayTime = 0;
        this.serverSampleRate = 22050;
        this.audioStreamEnded = true;
        this.isPlayingAudio = false;

        // Settings & Session (persisted in localStorage)
        this.playRobotAudio = localStorage.getItem('relobot_chat_robot_audio') !== 'false';
        this.playBrowserAudio = localStorage.getItem('relobot_chat_browser_audio') !== 'false';
        this.conversationId = localStorage.getItem('relobot_chat_conv_id') || null;

        // Event callbacks
        this.onToken = null;            // (token, msgId, convId)
        this.onStart = null;            // (msgId, convId)
        this.onInit = null;             // (convId, msgId)
        this.onDone = null;             // (fullText, msgId, convId)
        this.onError = null;            // (errorMsg, msgId, convId)
        this.onStatus = null;           // (statusObj)
        this.onConnectionChange = null; // (isConnected)
        this.onAudioPlaying = null;     // (isPlaying)
    }

    init() {
        this._connect();
    }

    _getWsUrl() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        return `${protocol}//${window.location.host}/chat-ws/`;
    }

    _connect() {
        if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
            return;
        }

        const url = this._getWsUrl();
        console.log(`[ChatService] Connecting to AI voice chat server at ${url}...`);

        try {
            this.ws = new WebSocket(url);
            this.ws.binaryType = 'arraybuffer';

            this.ws.onopen = () => {
                console.log('[ChatService] WebSocket connected successfully.');
                this.isConnected = true;
                this.isReconnecting = false;
                if (this.reconnectTimer) {
                    clearTimeout(this.reconnectTimer);
                    this.reconnectTimer = null;
                }
                if (this.onConnectionChange) this.onConnectionChange(true);
                this.send({ type: 'status' });
            };

            this.ws.onmessage = (event) => {
                if (event.data instanceof ArrayBuffer) {
                    this._handleBinaryAudio(event.data);
                } else {
                    try {
                        const data = JSON.parse(event.data);
                        this._handleMessage(data);
                    } catch (e) {
                        console.warn('[ChatService] Failed to parse message:', e);
                    }
                }
            };

            this.ws.onerror = (err) => {
                console.warn('[ChatService] WebSocket error:', err);
            };

            this.ws.onclose = () => {
                console.log('[ChatService] WebSocket closed.');
                this.isConnected = false;
                if (this.onConnectionChange) this.onConnectionChange(false);
                this._scheduleReconnect();
            };
        } catch (e) {
            console.error('[ChatService] WebSocket initiation error:', e);
            this._scheduleReconnect();
        }
    }

    _scheduleReconnect() {
        if (this.isReconnecting) return;
        this.isReconnecting = true;
        this.reconnectTimer = setTimeout(() => {
            this.isReconnecting = false;
            this._connect();
        }, 3000);
    }

    _handleMessage(data) {
        const type = data.type;
        const msgId = data.msg_id;
        const convId = data.conversation_id;

        if (convId) {
            this._setPinnedConversationId(convId);
        }

        switch (type) {
            case 'start':
                if (this.onStart) this.onStart(msgId, convId || this.conversationId);
                break;

            case 'init':
                if (this.onInit) this.onInit(convId, msgId);
                break;

            case 'token':
                if (this.onToken) this.onToken(data.text, msgId, convId || this.conversationId);
                break;

            case 'done':
                if (this.onDone) this.onDone(data.full_text, msgId, convId || this.conversationId);
                break;

            case 'error':
                console.error(`[ChatService] AGY server error for [${msgId}]:`, data.error);
                if (this.onError) this.onError(data.error, msgId, convId || this.conversationId);
                break;

            case 'status':
                if (this.onStatus) this.onStatus(data);
                break;

            case 'audio_start':
                this.serverSampleRate = data.sample_rate || 22050;
                this._initAudioStream();
                break;

            case 'audio_end':
                this._endAudioStream();
                break;

            default:
                break;
        }
    }

    _setPinnedConversationId(convId) {
        if (!convId || convId === this.conversationId) return;
        this.conversationId = convId;
        try {
            localStorage.setItem('relobot_chat_conv_id', convId);
        } catch (e) {
            console.warn('[ChatService] Could not persist conversationId:', e);
        }
    }

    send(msgObj) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(msgObj));
            return true;
        }
        return false;
    }

    sendPrompt(text, msgId = `msg_${Date.now()}`) {
        const payload = {
            type: 'prompt',
            text: text,
            msg_id: msgId,
            conversation_id: this.conversationId,
            play_robot_audio: this.playRobotAudio,
            stream_browser_audio: this.playBrowserAudio
        };
        return this.send(payload);
    }

    cancelPrompt() {
        this.send({ type: 'cancel' });
        this._stopAudioPlayback();
    }

    flushConversation() {
        this.conversationId = null;
        try {
            localStorage.removeItem('relobot_chat_conv_id');
            localStorage.removeItem('relobot_chat_messages');
        } catch (e) {
            console.warn('[ChatService] Failed to clear localStorage:', e);
        }
        this._stopAudioPlayback();
    }

    setPlayRobotAudio(enabled) {
        this.playRobotAudio = Boolean(enabled);
        localStorage.setItem('relobot_chat_robot_audio', this.playRobotAudio.toString());
    }

    setPlayBrowserAudio(enabled) {
        this.playBrowserAudio = Boolean(enabled);
        localStorage.setItem('relobot_chat_browser_audio', this.playBrowserAudio.toString());
        if (!this.playBrowserAudio) {
            this._stopAudioPlayback();
        }
    }

    // --- Web Audio PCM Streaming ---

    _ensureAudioContext() {
        if (!this.audioContext) {
            const AudioCtx = window.AudioContext || window.webkitAudioContext;
            this.audioContext = new AudioCtx();
        }
        if (this.audioContext.state === 'suspended') {
            this.audioContext.resume();
        }
    }

    _initAudioStream() {
        this._ensureAudioContext();
        this.audioStreamEnded = false;
        this.nextPlayTime = 0;
    }

    _endAudioStream() {
        this.audioStreamEnded = true;
        if (this.activeAudioSources.length === 0) {
            this.isPlayingAudio = false;
            if (this.onAudioPlaying) this.onAudioPlaying(false);
        }
    }

    _handleBinaryAudio(arrayBuffer) {
        if (!this.playBrowserAudio || !arrayBuffer || arrayBuffer.byteLength === 0) return;
        try {
            this._ensureAudioContext();
            const int16Array = new Int16Array(arrayBuffer);
            const numSamples = int16Array.length;
            if (numSamples === 0) return;

            const sampleRate = this.serverSampleRate || 22050;
            const audioBuffer = this.audioContext.createBuffer(1, numSamples, sampleRate);
            const channelData = audioBuffer.getChannelData(0);

            // Normalize Int16 [-32768, 32767] to Float32 [-1.0, 1.0]
            for (let i = 0; i < numSamples; i++) {
                channelData[i] = int16Array[i] / 32768.0;
            }

            const currentTime = this.audioContext.currentTime;
            if (!this.nextPlayTime || this.nextPlayTime < currentTime) {
                this.nextPlayTime = currentTime + 0.025; // 25ms lead-in to prevent underrun
            }

            const source = this.audioContext.createBufferSource();
            source.buffer = audioBuffer;
            source.connect(this.audioContext.destination);
            source.start(this.nextPlayTime);
            this.nextPlayTime += audioBuffer.duration;

            this.activeAudioSources.push(source);
            if (!this.isPlayingAudio) {
                this.isPlayingAudio = true;
                if (this.onAudioPlaying) this.onAudioPlaying(true);
            }

            source.onended = () => {
                const idx = this.activeAudioSources.indexOf(source);
                if (idx !== -1) this.activeAudioSources.splice(idx, 1);
                if (this.activeAudioSources.length === 0 && this.audioStreamEnded) {
                    this.isPlayingAudio = false;
                    if (this.onAudioPlaying) this.onAudioPlaying(false);
                }
            };
        } catch (e) {
            console.warn('[ChatService] Error playing binary PCM frame:', e);
        }
    }

    _stopAudioPlayback() {
        for (const src of this.activeAudioSources) {
            try {
                src.stop();
                src.disconnect();
            } catch (e) {}
        }
        this.activeAudioSources = [];
        this.nextPlayTime = 0;
        this.audioStreamEnded = true;
        this.isPlayingAudio = false;
        if (this.onAudioPlaying) this.onAudioPlaying(false);
    }

    destroy() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
        if (this.ws) {
            this.ws.onclose = null;
            this.ws.close();
            this.ws = null;
        }
        this._stopAudioPlayback();
        if (this.audioContext) {
            this.audioContext.close().catch(() => {});
            this.audioContext = null;
        }
    }
}
