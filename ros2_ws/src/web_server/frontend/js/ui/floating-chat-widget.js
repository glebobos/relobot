/**
 * ReloBot Floating Support Chat Widget Controller
 * Embedded chat popover for Map & Camera screens.
 * Provides voice transcription (STT) without Piper TTS playback.
 */

import { chatService } from '../services/chat-service.js';
import { VoiceRecognitionService } from '../services/voice-recognition-service.js';

const STORAGE_CHAT_MESSAGES_KEY = 'relobot_chat_messages';

export class FloatingChatWidget {
    constructor() {
        this.chatService = chatService;
        this.voiceService = new VoiceRecognitionService();

        // DOM elements
        this.widget = document.getElementById('floatingChatWidget');
        this.mapToggleBtn = document.getElementById('map-chat-toggle-btn');
        this.cameraToggleBtn = document.getElementById('camera-chat-toggle-btn');
        this.closeBtn = document.getElementById('floatingChatCloseBtn');
        this.flushBtn = document.getElementById('floatingChatFlushBtn');
        this.feed = document.getElementById('floatingChatFeed');
        this.input = document.getElementById('floatingChatInputText');
        this.sendBtn = document.getElementById('floatingChatSendBtn');
        this.micBtn = document.getElementById('floatingChatMicBtn');
        this.langBtn = document.getElementById('floatingChatLangBtn');
        this.langLabel = document.getElementById('floatingChatLangLabel');
        this.liveBox = document.getElementById('floatingChatLiveBox');
        this.liveText = document.getElementById('floatingChatLiveText');
        this.waveform = document.getElementById('floatingChatWaveform');
        this.statusDot = document.getElementById('floatingChatStatusDot');

        // State
        this.isOpen = false;
        this.currentBotMsgBubble = null;
        this.currentBotMsgText = '';
        this.activeMsgId = null;
        this.isGenerating = false;
        this.savedMessages = [];
        this.unsubscribers = [];
        this.domListeners = [];
    }

    init() {
        this._loadMessageHistory();
        this._updateLanguageUI();
        this._setupServiceEvents();
        this._setupVoiceEvents();
        this._setupDomListeners();
        this._setupEventContainment();
    }

    _setupServiceEvents() {
        this.unsubscribers.push(
            this.chatService.on('connectionChange', (connected) => {
                this._updateStatusDot(connected ? 'connected' : 'disconnected');
            }),

            this.chatService.on('start', (msgId) => {
                this.isGenerating = true;
                this.activeMsgId = msgId;
                this._createBotMessageBubble(msgId);
            }),

            this.chatService.on('statusUpdate', (statusText, msgId) => {
                const targetId = msgId || this.activeMsgId;
                const statusElem = targetId ? document.getElementById(`fc_status_${targetId}`) : null;
                if (statusElem && !this.currentBotMsgText) {
                    const textSpan = statusElem.querySelector('.c-chat-msg__status-text');
                    if (textSpan) {
                        textSpan.textContent = statusText;
                    }
                }
            }),

            this.chatService.on('token', (token, msgId) => {
                if (this.currentBotMsgBubble && this.activeMsgId === msgId) {
                    this.currentBotMsgText += token;
                    this._updateBotMessageBubble(this.currentBotMsgText);
                    this._scrollToBottom();
                }
            }),

            this.chatService.on('done', (fullText, msgId) => {
                this.isGenerating = false;
                const finalText = fullText || this.currentBotMsgText;
                if (this.currentBotMsgBubble && this.activeMsgId === msgId) {
                    this.currentBotMsgText = finalText;
                    this._updateBotMessageBubble(this.currentBotMsgText);
                    this._scrollToBottom();
                }

                if (finalText) {
                    this._saveMessageToHistory({
                        role: 'bot',
                        text: finalText,
                        time: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
                        msgId: msgId
                    });
                }

                this.currentBotMsgBubble = null;
                this.currentBotMsgText = '';
                this.activeMsgId = null;
            }),

            this.chatService.on('error', (errorMsg, msgId) => {
                this.isGenerating = false;
                if (this.currentBotMsgBubble) {
                    this._updateBotMessageBubble(`${this.currentBotMsgText}\n\n*[Error: ${errorMsg}]*`);
                }
                this.currentBotMsgBubble = null;
                this.activeMsgId = null;
            })
        );
    }

    _setupVoiceEvents() {
        this.voiceService.onStart = () => {
            const lang = this.voiceService.getLanguageShort();
            if (this.micBtn) this.micBtn.classList.add('is-recording');
            if (this.liveBox) this.liveBox.classList.add('is-visible');
            if (this.waveform) this.waveform.classList.add('is-animating');
            if (this.liveText) this.liveText.textContent = `Listening (${lang})...`;
        };

        this.voiceService.onLanguageChange = () => {
            this._updateLanguageUI();
        };

        this.voiceService.onInterimResult = (transcript) => {
            if (this.liveText) this.liveText.textContent = `"${transcript}"`;
            if (this.input) {
                this.input.value = transcript;
                this._autoGrowInput();
            }
            this._updateSendButtonState();
        };

        this.voiceService.onFinalResult = (transcript) => {
            if (this.liveText) this.liveText.textContent = `"${transcript}"`;
            if (this.input) {
                this.input.value = transcript;
                this._autoGrowInput();
            }
            this._updateSendButtonState();

            // Auto-send on completed sentence
            if (transcript.trim()) {
                setTimeout(() => {
                    this.voiceService.stopRecording();
                    this._sendMessage(transcript.trim());
                }, 350);
            }
        };

        this.voiceService.onEnd = () => {
            if (this.micBtn) this.micBtn.classList.remove('is-recording');
            if (this.waveform) this.waveform.classList.remove('is-animating');
            setTimeout(() => {
                if (this.liveBox && !this.voiceService.isRecording) {
                    this.liveBox.classList.remove('is-visible');
                }
            }, 800);
        };

        this.voiceService.onError = (err) => {
            if (this.micBtn) this.micBtn.classList.remove('is-recording');
            if (this.waveform) this.waveform.classList.remove('is-animating');
            if (this.liveBox) this.liveBox.classList.add('is-visible');
            if (this.liveText) this.liveText.textContent = `${err}`;
            setTimeout(() => {
                if (this.liveBox && !this.voiceService.isRecording) {
                    this.liveBox.classList.remove('is-visible');
                }
            }, 4000);
        };
    }

    _setupDomListeners() {
        const add = (elem, evt, handler) => {
            if (!elem) return;
            elem.addEventListener(evt, handler);
            this.domListeners.push({ elem, evt, handler });
        };

        // Map button toggle
        add(this.mapToggleBtn, 'click', (e) => {
            e.stopPropagation();
            this.toggle();
        });

        // Camera button toggle
        add(this.cameraToggleBtn, 'click', (e) => {
            e.stopPropagation();
            this.toggle();
        });

        // Close button
        add(this.closeBtn, 'click', (e) => {
            e.stopPropagation();
            this.close();
        });

        // Flush button
        add(this.flushBtn, 'click', (e) => {
            e.stopPropagation();
            this._flushConversation();
        });

        // Mic button
        add(this.micBtn, 'click', (e) => {
            e.stopPropagation();
            if (this.voiceService.isRecording) {
                this.voiceService.stopRecording();
            } else {
                this.voiceService.startRecording();
            }
        });

        // Language toggle
        add(this.langBtn, 'click', (e) => {
            e.stopPropagation();
            this.voiceService.toggleLanguage();
            this._updateLanguageUI();
        });

        // Send button
        add(this.sendBtn, 'click', (e) => {
            e.stopPropagation();
            const text = this.input ? this.input.value.trim() : '';
            if (text) this._sendMessage(text);
        });

        // Input textarea handlers
        if (this.input) {
            add(this.input, 'input', () => {
                this._autoGrowInput();
                this._updateSendButtonState();
            });

            add(this.input, 'keydown', (e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault();
                    const text = this.input.value.trim();
                    if (text) this._sendMessage(text);
                }
            });
        }

        // Close on Escape
        add(document, 'keydown', (e) => {
            if (e.key === 'Escape' && this.isOpen) {
                this.close();
            }
        });

        // Close on outside click (if clicking outside widget and outside toggle buttons)
        add(document, 'pointerdown', (e) => {
            if (!this.isOpen) return;
            const clickedInside = this.widget && this.widget.contains(e.target);
            const clickedToggle = (this.mapToggleBtn && this.mapToggleBtn.contains(e.target)) ||
                                  (this.cameraToggleBtn && this.cameraToggleBtn.contains(e.target));
            if (!clickedInside && !clickedToggle) {
                this.close();
            }
        });

        // Screen changed: hide floating chat if switching to AI Chat (Screen 2) or Settings (Screen 3)
        add(window, 'screenChanged', (e) => {
            const screenIndex = e.detail?.index;
            if (screenIndex === 2 || screenIndex === 3) {
                if (this.isOpen) this.close();
            }
        });
    }

    _setupEventContainment() {
        if (!this.widget) return;
        // Shield underlying 3D Map (OrbitControls) and Camera joystick from widget pointer events
        const events = ['pointerdown', 'mousedown', 'touchstart', 'touchmove', 'touchend', 'wheel'];
        events.forEach((evt) => {
            this.widget.addEventListener(evt, (e) => {
                e.stopPropagation();
            }, { passive: evt.startsWith('touch') });
        });
    }

    toggle() {
        if (this.isOpen) {
            this.close();
        } else {
            this.open();
        }
    }

    open() {
        if (!this.widget) return;
        this.isOpen = true;
        this.widget.style.display = 'flex';
        this._updateToggleButtons(true);
        this._scrollToBottom();
        if (this.input) {
            setTimeout(() => this.input.focus(), 150);
        }
    }

    close() {
        if (!this.widget) return;
        this.isOpen = false;
        this.widget.style.display = 'none';
        this._updateToggleButtons(false);
        if (this.voiceService.isRecording) {
            this.voiceService.stopRecording();
        }
    }

    _updateToggleButtons(active) {
        if (this.mapToggleBtn) this.mapToggleBtn.classList.toggle('is-active', active);
        if (this.cameraToggleBtn) this.cameraToggleBtn.classList.toggle('is-active', active);
    }

    _updateStatusDot(state) {
        if (!this.statusDot) return;
        this.statusDot.classList.remove('is-connected', 'is-disconnected');
        if (state) this.statusDot.classList.add(`is-${state}`);
    }

    _updateLanguageUI() {
        const lang = this.voiceService.getLanguageShort();
        if (this.langLabel) this.langLabel.textContent = lang;
        if (this.langBtn) {
            this.langBtn.title = `Voice Input: ${lang === 'RU' ? 'Russian' : 'English'} (Click to toggle)`;
        }
    }

    _sendMessage(text) {
        if (!text) return;
        const timeStr = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

        this._createUserMessageBubble(text, timeStr);
        this._saveMessageToHistory({
            role: 'user',
            text: text,
            time: timeStr
        });

        if (this.input) {
            this.input.value = '';
            this.input.style.height = 'auto';
        }
        this._updateSendButtonState();
        this._scrollToBottom();

        // Dispatch with zero Piper audio playback
        this.chatService.sendPrompt(text, `msg_${Date.now()}`, {
            play_robot_audio: false,
            stream_browser_audio: false
        });
    }

    _createUserMessageBubble(text, timeStr) {
        if (!this.feed) return;
        const msgDiv = document.createElement('div');
        msgDiv.className = 'c-chat-msg c-chat-msg--user';
        msgDiv.innerHTML = `
            <div class="c-chat-msg__avatar"><i class="fas fa-user"></i></div>
            <div class="c-chat-msg__content">
                <div class="c-chat-msg__bubble">${this._escapeHtml(text)}</div>
                <div class="c-chat-msg__time">${timeStr}</div>
            </div>
        `;
        this.feed.appendChild(msgDiv);
    }

    _createBotMessageBubble(msgId, timeStr = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })) {
        if (!this.feed) return;
        const msgDiv = document.createElement('div');
        msgDiv.className = 'c-chat-msg c-chat-msg--bot';
        msgDiv.id = `fc_bot_${msgId}`;
        msgDiv.innerHTML = `
            <div class="c-chat-msg__avatar"><i class="fas fa-robot"></i></div>
            <div class="c-chat-msg__content">
                <div class="c-chat-msg__bubble" id="fc_bubble_${msgId}">
                    <div class="c-chat-msg__status" id="fc_status_${msgId}">
                        <span class="c-chat-msg__status-icon"><i class="fas fa-circle-notch fa-spin"></i></span>
                        <span class="c-chat-msg__status-text">Thinking...</span>
                    </div>
                </div>
                <div class="c-chat-msg__time">
                    <span>${timeStr}</span>
                </div>
            </div>
        `;
        this.feed.appendChild(msgDiv);
        this.currentBotMsgBubble = document.getElementById(`fc_bubble_${msgId}`);
        this.currentBotMsgText = '';
        this._scrollToBottom();
    }

    _updateBotMessageBubble(rawText) {
        if (!this.currentBotMsgBubble) return;
        this.currentBotMsgBubble.innerHTML = this._renderMarkdown(rawText);
    }

    _renderMarkdown(text) {
        if (!text) return '';
        let html = this._escapeHtml(text);
        html = html.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
        html = html.replace(/\*([^*]+)\*/g, '<em>$1</em>');
        html = html.replace(/`([^`]+)`/g, '<code class="c-chat-code">$1</code>');
        html = html.replace(/\n/g, '<br>');
        return html;
    }

    _escapeHtml(str) {
        const div = document.createElement('div');
        div.textContent = str;
        return div.innerHTML;
    }

    _loadMessageHistory() {
        try {
            const raw = localStorage.getItem(STORAGE_CHAT_MESSAGES_KEY);
            if (!raw) return;
            const messages = JSON.parse(raw);
            if (Array.isArray(messages) && messages.length > 0) {
                this.savedMessages = messages;
                // Render the last 10 messages in the compact widget
                const recent = messages.slice(-10);
                recent.forEach((msg) => {
                    if (msg.role === 'user') {
                        this._createUserMessageBubble(msg.text, msg.time);
                    } else if (msg.role === 'bot') {
                        this._createBotMessageBubble(msg.msgId || `hist_${Math.random()}`, msg.time);
                        this._updateBotMessageBubble(msg.text);
                    }
                });
                this.currentBotMsgBubble = null;
                this._scrollToBottom();
            }
        } catch (e) {
            console.warn('[FloatingChatWidget] Could not load message history:', e);
        }
    }

    _saveMessageToHistory(msgObj) {
        try {
            this.savedMessages.push(msgObj);
            if (this.savedMessages.length > 50) {
                this.savedMessages = this.savedMessages.slice(-50);
            }
            localStorage.setItem(STORAGE_CHAT_MESSAGES_KEY, JSON.stringify(this.savedMessages));
        } catch (e) {
            console.warn('[FloatingChatWidget] Failed to save message:', e);
        }
    }

    _flushConversation() {
        this.chatService.flushConversation();
        this.savedMessages = [];

        if (this.feed) {
            this.feed.innerHTML = `
                <div class="c-floating-chat-widget__welcome">
                    <div class="c-floating-chat-widget__welcome-icon"><i class="fas fa-robot"></i></div>
                    <div class="c-floating-chat-widget__welcome-title">ReloBot Assistant</div>
                    <div class="c-floating-chat-widget__welcome-text">Ask questions, plan moves, or tap the microphone to dictate hands-free.</div>
                </div>
            `;
        }
    }

    _autoGrowInput() {
        if (!this.input) return;
        this.input.style.height = 'auto';
        this.input.style.height = `${Math.min(this.input.scrollHeight, 80)}px`;
    }

    _updateSendButtonState() {
        if (!this.sendBtn || !this.input) return;
        const hasText = this.input.value.trim().length > 0;
        this.sendBtn.classList.toggle('is-ready', hasText);
    }

    _scrollToBottom() {
        if (this.feed) {
            this.feed.scrollTop = this.feed.scrollHeight;
        }
    }

    destroy() {
        this.close();
        this.voiceService.destroy();
        this.domListeners.forEach(({ elem, evt, handler }) => {
            elem.removeEventListener(evt, handler);
        });
        this.domListeners = [];
        this.unsubscribers.forEach((unsub) => {
            if (typeof unsub === 'function') unsub();
        });
        this.unsubscribers = [];
    }
}
