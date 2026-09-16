/**
 * ReloBot Voice Recognition Service
 * Direct Web Speech API integration supporting bilingual voice control (English / Russian).
 * Optimized for mobile and desktop browsers with clean permission and error handling.
 */

export class VoiceRecognitionService {
    constructor() {
        const SpeechRec = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.hasSpeechRec = Boolean(SpeechRec);
        this.isSecure = window.isSecureContext || window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';

        // Language setting ('en-US' or 'ru-RU')
        const storedLang = localStorage.getItem('relobot_chat_voice_lang');
        if (storedLang) {
            this.lang = storedLang;
        } else {
            this.lang = navigator.language && navigator.language.startsWith('ru') ? 'ru-RU' : 'en-US';
        }

        this.recognition = null;
        this.isRecording = false;

        // Callbacks
        this.onInterimResult = null; // (transcript)
        this.onFinalResult = null;   // (transcript)
        this.onStart = null;         // ()
        this.onEnd = null;           // ()
        this.onError = null;         // (errorMessage)
        this.onLanguageChange = null;// (lang)
    }

    setLanguage(lang) {
        if (lang === 'ru' || lang === 'ru-RU') {
            this.lang = 'ru-RU';
        } else {
            this.lang = 'en-US';
        }
        try {
            localStorage.setItem('relobot_chat_voice_lang', this.lang);
        } catch (e) {}
        if (this.onLanguageChange) this.onLanguageChange(this.lang);
        return this.lang;
    }

    getLanguage() {
        return this.lang;
    }

    getLanguageShort() {
        return this.lang.startsWith('ru') ? 'RU' : 'EN';
    }

    toggleLanguage() {
        const nextLang = this.lang.startsWith('ru') ? 'en-US' : 'ru-RU';
        return this.setLanguage(nextLang);
    }

    _createRecognition() {
        const SpeechRec = window.SpeechRecognition || window.webkitSpeechRecognition;
        if (!SpeechRec) return null;

        try {
            const rec = new SpeechRec();
            rec.continuous = false;
            rec.interimResults = true;
            rec.maxAlternatives = 1;
            rec.lang = this.lang;

            rec.onstart = () => {
                this.isRecording = true;
                if (this.onStart) this.onStart();
            };

            rec.onresult = (event) => {
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

            rec.onerror = (event) => {
                console.warn('[VoiceRec] SpeechRecognition error:', event.error);
                const friendlyMsg = this._formatErrorMessage(event.error);
                if (this.onError) this.onError(friendlyMsg);
            };

            rec.onend = () => {
                this.isRecording = false;
                if (this.onEnd) this.onEnd();
            };

            return rec;
        } catch (e) {
            console.error('[VoiceRec] Failed to construct SpeechRecognition:', e);
            return null;
        }
    }

    _formatErrorMessage(error) {
        if (typeof error === 'string') {
            switch (error) {
                case 'not-allowed':
                case 'permission-denied':
                    return 'Microphone access denied. Please allow microphone permissions in your browser.';
                case 'no-speech':
                    return 'No speech detected. Tap the mic and speak.';
                case 'audio-capture':
                    return 'Microphone unavailable or in use by another application.';
                case 'network':
                    return 'Voice network error. Speech recognition requires HTTPS or internet connectivity.';
                case 'service-not-allowed':
                    return 'Speech recognition not permitted. Please access via HTTPS.';
                case 'aborted':
                    return 'Voice recognition stopped.';
                default:
                    return `Voice error: ${error}`;
            }
        }
        return error.message || 'Microphone error occurred.';
    }

    startRecording() {
        if (this.isRecording) return;

        if (!this.hasSpeechRec) {
            const msg = !this.isSecure
                ? 'Speech recognition requires a secure HTTPS connection.'
                : 'Speech recognition is not supported in this browser.';
            console.warn('[VoiceRec]', msg);
            if (this.onError) this.onError(msg);
            return;
        }

        try {
            if (this.recognition) {
                try {
                    this.recognition.abort();
                } catch (e) {}
                this.recognition = null;
            }

            this.recognition = this._createRecognition();
            if (!this.recognition) {
                if (this.onError) this.onError('Failed to initialize speech recognition.');
                return;
            }

            this.recognition.start();
        } catch (e) {
            console.error('[VoiceRec] Exception in startRecording:', e);
            const msg = this._formatErrorMessage(e);
            if (this.onError) this.onError(msg);
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
    }

    destroy() {
        this.stopRecording();
        if (this.recognition) {
            try {
                this.recognition.abort();
            } catch (e) {}
            this.recognition = null;
        }
    }
}
