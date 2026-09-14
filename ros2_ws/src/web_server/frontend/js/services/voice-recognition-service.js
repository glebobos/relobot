/**
 * ReloBot Voice Recognition & Audio Analyser Service
 * Uses Web Speech API for real-time live transcription and Web Audio API for audio visualizer waveforms.
 */

export class VoiceRecognitionService {
    constructor() {
        const SpeechRec = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.hasSpeechRec = Boolean(SpeechRec);
        this.recognition = SpeechRec ? new SpeechRec() : null;
        
        if (this.recognition) {
            this.recognition.continuous = false;
            this.recognition.interimResults = true;
            this.recognition.lang = navigator.language || 'en-US';
        }

        this.isRecording = false;
        this.audioContext = null;
        this.analyser = null;
        this.mediaStream = null;
        this.animationFrameId = null;

        // Callbacks
        this.onInterimResult = null;
        this.onFinalResult = null;
        this.onAudioVolume = null;
        this.onStart = null;
        this.onEnd = null;
        this.onError = null;

        this._setupRecognitionEvents();
    }

    _setupRecognitionEvents() {
        if (!this.recognition) return;

        this.recognition.onstart = () => {
            this.isRecording = true;
            if (this.onStart) this.onStart();
        };

        this.recognition.onresult = (event) => {
            let interimTranscript = '';
            let finalTranscript = '';

            for (let i = event.resultIndex; i < event.results.length; ++i) {
                const transcript = event.results[i][0].transcript;
                if (event.results[i].isFinal) {
                    finalTranscript += transcript;
                } else {
                    interimTranscript += transcript;
                }
            }

            if (interimTranscript && this.onInterimResult) {
                this.onInterimResult(interimTranscript);
            }
            if (finalTranscript && this.onFinalResult) {
                this.onFinalResult(finalTranscript);
            }
        };

        this.recognition.onerror = (event) => {
            console.warn('[VoiceRec] Error:', event.error);
            if (this.onError) this.onError(event.error);
        };

        this.recognition.onend = () => {
            this.isRecording = false;
            this._stopAudioAnalyser();
            if (this.onEnd) this.onEnd();
        };
    }

    async startRecording() {
        if (this.isRecording) return;
        if (!this.recognition) {
            console.warn('[VoiceRec] Speech recognition not supported in this browser.');
            return;
        }

        try {
            // Start Web Audio analyser for volume waveform visualization
            await this._startAudioAnalyser();
            this.recognition.start();
        } catch (e) {
            console.error('[VoiceRec] Failed to start speech recognition:', e);
            if (this.onError) this.onError(e);
        }
    }

    stopRecording() {
        if (!this.isRecording) return;
        try {
            if (this.recognition) {
                this.recognition.stop();
            }
        } catch (e) {
            console.warn('[VoiceRec] Stop error:', e);
        }
        this.isRecording = false;
        this._stopAudioAnalyser();
    }

    async _startAudioAnalyser() {
        try {
            if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) return;
            this.mediaStream = await navigator.mediaDevices.getUserMedia({ audio: true });
            
            const AudioCtx = window.AudioContext || window.webkitAudioContext;
            this.audioContext = new AudioCtx();
            const source = this.audioContext.createMediaStreamSource(this.mediaStream);
            this.analyser = this.audioContext.createAnalyser();
            this.analyser.fftSize = 64;
            source.connect(this.analyser);

            const dataArray = new Uint8Array(this.analyser.frequencyBinCount);

            const checkVolume = () => {
                if (!this.isRecording || !this.analyser) return;
                this.analyser.getByteFrequencyData(dataArray);
                let sum = 0;
                for (let i = 0; i < dataArray.length; i++) {
                    sum += dataArray[i];
                }
                const average = sum / dataArray.length;
                const volumeNormalized = Math.min(1.0, average / 128.0);
                if (this.onAudioVolume) {
                    this.onAudioVolume(volumeNormalized);
                }
                this.animationFrameId = requestAnimationFrame(checkVolume);
            };
            this.animationFrameId = requestAnimationFrame(checkVolume);
        } catch (e) {
            console.warn('[VoiceRec] Audio analyser mic access error:', e);
        }
    }

    _stopAudioAnalyser() {
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
            this.animationFrameId = null;
        }
        if (this.mediaStream) {
            this.mediaStream.getTracks().forEach(t => t.stop());
            this.mediaStream = null;
        }
        if (this.audioContext) {
            this.audioContext.close().catch(() => {});
            this.audioContext = null;
        }
        this.analyser = null;
        if (this.onAudioVolume) {
            this.onAudioVolume(0);
        }
    }

    destroy() {
        this.stopRecording();
        this._stopAudioAnalyser();
    }
}
