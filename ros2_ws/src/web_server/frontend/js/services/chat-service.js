/**
 * ReloBot AI Chat & WebSocket Service
 * Handles bidirectional WebSocket connection with the voice chat server,
 * streaming tokens, status updates, and browser-side Web Audio playback.
 */

export class ChatService {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.isReconnecting = false;
        this.reconnectTimer = null;

        // Web Audio for browser-side speech playback
        this.audioContext = null;
        this.audioQueue = [];
        this.isPlayingAudio = false;

        // Settings (persisted in localStorage)
        this.playRobotAudio = localStorage.getItem('relobot_chat_robot_audio') !== 'false';
        this.playBrowserAudio = localStorage.getItem('relobot_chat_browser_audio') !== 'false';

        // Event callbacks
        this.onToken = null;         // (token, msgId)
        this.onStart = null;         // (msgId)
        this.onDone = null;          // (fullText, msgId)
        this.onError = null;         // (errorMsg, msgId)
        this.onStatus = null;        // (statusObj)
        this.onConnectionChange = null; // (isConnected)
        this.onAudioPlaying = null;  // (isPlaying)
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
                try {
                    const data = JSON.parse(event.data);
                    this._handleMessage(data);
                } catch (e) {
                    console.warn('[ChatService] Failed to parse incoming message:', e);
                }
            };

            this.ws.onerror = (err) => {
                console.warn('[ChatService] WebSocket error:', err);
            };

            this.ws.onclose = () => {
                console.log('[ChatService] WebSocket connection closed.');
                this.isConnected = false;
                if (this.onConnectionChange) this.onConnectionChange(false);
                this._scheduleReconnect();
            };
        } catch (e) {
            console.error('[ChatService] Error initiating WebSocket:', e);
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

        switch (type) {
            case 'start':
                if (this.onStart) this.onStart(msgId);
                break;

            case 'token':
                if (this.onToken) this.onToken(data.text, msgId);
                break;

            case 'done':
                if (this.onDone) this.onDone(data.full_text, msgId);
                break;

            case 'error':
                if (this.onError) this.onError(data.error, msgId);
                break;

            case 'status':
                if (this.onStatus) this.onStatus(data);
                break;

            case 'audio':
                if (this.playBrowserAudio && data.audio) {
                    this._queueAudioChunk(data.audio);
                }
                break;

            default:
                break;
        }
    }

    send(msgObj) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(msgObj));
            return true;
        } else {
            console.warn('[ChatService] Cannot send message, WebSocket not connected.');
            return false;
        }
    }

    sendPrompt(text, msgId = `msg_${Date.now()}`) {
        const payload = {
            type: 'prompt',
            text: text,
            msg_id: msgId,
            play_robot_audio: this.playRobotAudio,
            stream_browser_audio: this.playBrowserAudio
        };
        return this.send(payload);
    }

    cancelPrompt() {
        this.send({ type: 'cancel' });
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

    // --- Web Audio Playback Queue ---

    _ensureAudioContext() {
        if (!this.audioContext) {
            const AudioCtx = window.AudioContext || window.webkitAudioContext;
            this.audioContext = new AudioCtx();
        }
        if (this.audioContext.state === 'suspended') {
            this.audioContext.resume();
        }
    }

    async _queueAudioChunk(base64Wav) {
        try {
            this._ensureAudioContext();
            const binaryStr = atob(base64Wav);
            const len = binaryStr.length;
            const bytes = new Uint8Array(len);
            for (let i = 0; i < len; i++) {
                bytes[i] = binaryStr.charCodeAt(i);
            }

            const audioBuffer = await this.audioContext.decodeAudioData(bytes.buffer.slice(0));
            this.audioQueue.push(audioBuffer);

            if (!this.isPlayingAudio) {
                this._playNextAudioChunk();
            }
        } catch (e) {
            console.warn('[ChatService] Failed to decode audio chunk:', e);
        }
    }

    _playNextAudioChunk() {
        if (this.audioQueue.length === 0) {
            this.isPlayingAudio = false;
            if (this.onAudioPlaying) this.onAudioPlaying(false);
            return;
        }

        this.isPlayingAudio = true;
        if (this.onAudioPlaying) this.onAudioPlaying(true);

        const audioBuffer = this.audioQueue.shift();
        const source = this.audioContext.createBufferSource();
        source.buffer = audioBuffer;
        source.connect(this.audioContext.destination);

        source.onended = () => {
            this._playNextAudioChunk();
        };

        source.start(0);
    }

    _stopAudioPlayback() {
        this.audioQueue = [];
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
