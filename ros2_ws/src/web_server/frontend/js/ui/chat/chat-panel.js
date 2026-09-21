/**
 * ReloBot AI Chat Panel Controller
 * Coordinates UI message rendering, session persistence, voice transcription, and token streaming.
 */

import { ChatService, chatService } from '../../services/chat-service.js';
import { VoiceRecognitionService } from '../../services/voice-recognition-service.js';

const STORAGE_CHAT_MESSAGES_KEY = 'relobot_chat_messages';

export class ChatPanel {
    constructor(chatServiceInstance = chatService) {
        this.chatService = chatServiceInstance;
        this.voiceService = new VoiceRecognitionService();

        // DOM elements
        this.feed = document.getElementById('chatFeed');
        this.input = document.getElementById('chatInputText');
        this.sendBtn = document.getElementById('chatSendBtn');
        this.micBtn = document.getElementById('chatMicBtn');
        this.liveBox = document.getElementById('chatLiveBox');
        this.liveText = document.getElementById('chatLiveText');
        this.waveform = document.getElementById('chatWaveform');
        this.statusDot = document.getElementById('chatStatusDot');
        this.sessionChip = document.getElementById('chatSessionChip');
        this.sessionIdSpan = document.getElementById('chatSessionId');
        this.toggleRobotSpeakerBtn = document.getElementById('toggleRobotSpeaker');
        this.toggleBrowserAudioBtn = document.getElementById('toggleBrowserAudio');
        this.langBtn = document.getElementById('toggleChatLang');
        this.langLabel = document.getElementById('chatLangLabel');
        this.flushBtn = document.getElementById('chatFlushBtn');
        this.chatView = document.querySelector('.c-chat-view');
        this.advancedBtn = document.getElementById('toggleChatAdvanced');
        this.terminalView = document.getElementById('chatTerminalView');
        this.terminalFrame = document.getElementById('chatTerminalFrame');

        // State
        this.currentBotMsgBubble = null;
        this.currentBotMsgText = '';
        this.activeMsgId = null;
        this.isGenerating = false;
        this.savedMessages = [];
        this.isAdvancedMode = localStorage.getItem('relobot_chat_advanced_mode') === 'true';
        this.unsubscribers = [];
    }

    init() {
        this.chatService.init();
        this._loadMessageHistory();
        this._updateSessionBadge(this.chatService.conversationId);
        this._updateLanguageUI();
        this._setupChatServiceEvents();
        this._setupVoiceEvents();
        this._setupDomListeners();
        this._updateAudioToggleStates();
        this._updateAdvancedModeUI();
    }

    _setupChatServiceEvents() {
        this.unsubscribers.push(
            this.chatService.on('connectionChange', (connected) => {
                if (this.isAdvancedMode) {
                    this._updateStatusDot(connected ? 'advanced' : 'disconnected');
                    return;
                }
                this._updateStatusDot(connected ? 'connected' : 'disconnected');
            }),

            this.chatService.on('status', (status) => {
                if (this.isAdvancedMode) return;
                if (!status.has_agy) {
                    this._updateStatusDot('warning');
                }
            }),

            this.chatService.on('init', (convId) => {
                this._updateSessionBadge(convId);
            }),

            this.chatService.on('start', (msgId, convId) => {
                this.isGenerating = true;
                this.activeMsgId = msgId;
                if (convId) this._updateSessionBadge(convId);
                this._createBotMessageBubble(msgId);
            }),

            this.chatService.on('statusUpdate', (statusText, msgId) => {
                // Update inline status inside the bot bubble if still waiting for text
                const targetId = msgId || this.activeMsgId;
                const statusElem = targetId ? document.getElementById(`status_${targetId}`) : null;
                if (statusElem && !this.currentBotMsgText) {
                    const textSpan = statusElem.querySelector('.c-chat-msg__status-text');
                    if (textSpan) {
                        textSpan.textContent = statusText;
                    }
                }
            }),

            this.chatService.on('token', (token, msgId, convId) => {
                if (this.currentBotMsgBubble && this.activeMsgId === msgId) {
                    if (convId) this._updateSessionBadge(convId);
                    this.currentBotMsgText += token;
                    this._updateBotMessageBubble(this.currentBotMsgText, false);
                    this._scrollToBottom();
                }
            }),

            this.chatService.on('done', (fullText, msgId, convId) => {
                this.isGenerating = false;
                if (convId) this._updateSessionBadge(convId);

                const finalText = fullText || this.currentBotMsgText;
                if (this.currentBotMsgBubble && this.activeMsgId === msgId) {
                    this.currentBotMsgText = finalText;
                    this._updateBotMessageBubble(this.currentBotMsgText, true);
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

            this.chatService.onError = (errorMsg, msgId, convId) => {
                this.isGenerating = false;
                if (convId) this._updateSessionBadge(convId);
                if (this.currentBotMsgBubble) {
                    this._updateBotMessageBubble(`${this.currentBotMsgText}\n\n*[Error: ${errorMsg}]*`, true);
                }
                this.currentBotMsgBubble = null;
                this.activeMsgId = null;
            },

            this.chatService.on('error', (errorMsg, msgId, convId) => {
                this.isGenerating = false;
                if (convId) this._updateSessionBadge(convId);
                if (this.currentBotMsgBubble) {
                    this._updateBotMessageBubble(`${this.currentBotMsgText}\n\n*[Error: ${errorMsg}]*`, true);
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

            // Auto-dispatch prompt on final recognized sentence
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
        // Microphone button click
        if (this.micBtn) {
            this.micBtn.addEventListener('click', () => {
                if (this.voiceService.isRecording) {
                    this.voiceService.stopRecording();
                } else {
                    this.voiceService.startRecording();
                }
            });
        }

        // Language switch button
        if (this.langBtn) {
            this.langBtn.addEventListener('click', () => {
                this.voiceService.toggleLanguage();
                this._updateLanguageUI();
            });
        }

        // Send button click
        if (this.sendBtn) {
            this.sendBtn.addEventListener('click', () => {
                const text = this.input ? this.input.value.trim() : '';
                if (text) this._sendMessage(text);
            });
        }

        // Input textarea handlers
        if (this.input) {
            this.input.addEventListener('input', () => {
                this._autoGrowInput();
                this._updateSendButtonState();
            });

            this.input.addEventListener('keydown', (e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault();
                    const text = this.input.value.trim();
                    if (text) this._sendMessage(text);
                }
            });
        }

        // Audio toggles
        if (this.toggleRobotSpeakerBtn) {
            this.toggleRobotSpeakerBtn.addEventListener('click', () => {
                const next = !this.chatService.playRobotAudio;
                this.chatService.setPlayRobotAudio(next);
                this._updateAudioToggleStates();
            });
        }

        if (this.toggleBrowserAudioBtn) {
            this.toggleBrowserAudioBtn.addEventListener('click', () => {
                const next = !this.chatService.playBrowserAudio;
                this.chatService.setPlayBrowserAudio(next);
                this._updateAudioToggleStates();
            });
        }

        // Flush conversation button
        if (this.flushBtn) {
            this.flushBtn.addEventListener('click', () => {
                this._flushConversation();
            });
        }

        // Session chip copy ID handler
        if (this.sessionChip) {
            this.sessionChip.addEventListener('click', () => {
                this._copySessionId();
            });
        }

        // Advanced Mode Toggle button handler
        if (this.advancedBtn) {
            this.advancedBtn.addEventListener('click', () => {
                this._toggleAdvancedMode();
            });
        }
    }

    _toggleAdvancedMode() {
        this.isAdvancedMode = !this.isAdvancedMode;
        localStorage.setItem('relobot_chat_advanced_mode', this.isAdvancedMode ? 'true' : 'false');
        this._updateAdvancedModeUI();
    }

    _updateStatusDot(state) {
        if (!this.statusDot) return;
        this.statusDot.classList.remove('is-connected', 'is-disconnected', 'is-warning', 'is-advanced');
        if (state) {
            this.statusDot.classList.add(`is-${state}`);
        }
    }

    _updateAdvancedModeUI() {
        if (!this.chatView) return;

        this.chatView.classList.toggle('is-advanced-mode', this.isAdvancedMode);

        if (this.advancedBtn) {
            this.advancedBtn.classList.toggle('is-active', this.isAdvancedMode);
        }

        if (this.isAdvancedMode) {
            // Lazy-load terminal iframe if not loaded yet
            if (this.terminalFrame && !this.terminalFrame.getAttribute('src')) {
                this.terminalFrame.src = '/agy-terminal/';
            }
            this._updateStatusDot(this.chatService.isConnected ? 'advanced' : 'disconnected');
            if (this.voiceService && this.voiceService.isRecording) {
                this.voiceService.stopRecording();
            }
        } else {
            this._updateStatusDot(this.chatService.isConnected ? 'connected' : 'disconnected');
        }
    }

    _updateLanguageUI() {
        const lang = this.voiceService.getLanguageShort();
        if (this.langLabel) this.langLabel.textContent = lang;
        if (this.langBtn) {
            this.langBtn.title = `Voice Input Language: ${lang === 'RU' ? 'Russian (ru-RU)' : 'English (en-US)'} (Click to toggle)`;
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

        this.chatService.sendPrompt(text);
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
        msgDiv.id = `bot_${msgId}`;
        msgDiv.innerHTML = `
            <div class="c-chat-msg__avatar"><i class="fas fa-robot"></i></div>
            <div class="c-chat-msg__content">
                <div class="c-chat-msg__bubble" id="bubble_${msgId}">
                    <div class="c-chat-msg__status" id="status_${msgId}">
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
        this.currentBotMsgBubble = document.getElementById(`bubble_${msgId}`);
        this.currentBotMsgText = '';
        this._scrollToBottom();
    }

    _updateBotMessageBubble(rawText, _isFinal = false) {
        if (!this.currentBotMsgBubble) return;
        const rendered = this._renderMarkdown(rawText);
        this.currentBotMsgBubble.innerHTML = rendered;
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

    _updateSessionBadge(convId) {
        if (!this.sessionIdSpan || !this.sessionChip) return;
        if (convId && convId.trim()) {
            const shortId = convId.length > 8 ? `${convId.slice(0, 8)}...` : convId;
            this.sessionIdSpan.textContent = shortId;
            this.sessionChip.title = `Active AGY Session: ${convId}\nClick to copy full ID.`;
        } else {
            this.sessionIdSpan.textContent = 'New Session';
            this.sessionChip.title = 'No active conversation. Send a message to initialize an AGY session.';
        }
    }

    _copySessionId() {
        const convId = this.chatService.conversationId;
        if (!convId || !this.sessionChip || !this.sessionIdSpan) return;

        if (navigator.clipboard && navigator.clipboard.writeText) {
            navigator.clipboard.writeText(convId).then(() => {
                this.sessionChip.classList.add('is-copied');
                const prevText = this.sessionIdSpan.textContent;
                this.sessionIdSpan.textContent = 'Copied!';
                setTimeout(() => {
                    this.sessionChip.classList.remove('is-copied');
                    this.sessionIdSpan.textContent = prevText;
                }, 1500);
            }).catch(() => {});
        }
    }

    _loadMessageHistory() {
        try {
            const raw = localStorage.getItem(STORAGE_CHAT_MESSAGES_KEY);
            if (!raw) return;
            const messages = JSON.parse(raw);
            if (Array.isArray(messages) && messages.length > 0) {
                this.savedMessages = messages;
                messages.forEach((msg) => {
                    if (msg.role === 'user') {
                        this._createUserMessageBubble(msg.text, msg.time);
                    } else if (msg.role === 'bot') {
                        this._createBotMessageBubble(msg.msgId || `hist_${Math.random()}`, msg.time);
                        this._updateBotMessageBubble(msg.text, true);
                    }
                });
                this.currentBotMsgBubble = null;
                this._scrollToBottom();
            }
        } catch (e) {
            console.warn('[ChatPanel] Could not load message history:', e);
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
            console.warn('[ChatPanel] Failed to save message:', e);
        }
    }

    _flushConversation() {
        this.chatService.flushConversation();
        this.savedMessages = [];

        if (this.feed) {
            this.feed.innerHTML = `
                <div class="c-chat-welcome">
                    <div class="c-chat-welcome__icon"><i class="fas fa-robot"></i></div>
                    <div class="c-chat-welcome__title">ReloBot AI Assistant</div>
                    <div class="c-chat-welcome__text">Direct conversational interface powered by Antigravity (AGY) & Piper Neural Voice. Type a message or click the microphone to speak.</div>
                </div>
            `;
        }

        this._updateSessionBadge(null);
    }

    _autoGrowInput() {
        if (!this.input) return;
        this.input.style.height = 'auto';
        this.input.style.height = `${Math.min(this.input.scrollHeight, 100)}px`;
    }

    _updateSendButtonState() {
        if (!this.sendBtn || !this.input) return;
        const hasText = this.input.value.trim().length > 0;
        this.sendBtn.classList.toggle('is-ready', hasText);
    }

    _updateAudioToggleStates() {
        if (this.toggleRobotSpeakerBtn) {
            this.toggleRobotSpeakerBtn.classList.toggle('is-active', this.chatService.playRobotAudio);
        }
        if (this.toggleBrowserAudioBtn) {
            this.toggleBrowserAudioBtn.classList.toggle('is-active', this.chatService.playBrowserAudio);
        }
    }

    _scrollToBottom() {
        if (this.feed) {
            this.feed.scrollTop = this.feed.scrollHeight;
        }
    }

    destroy() {
        if (this.unsubscribers) {
            this.unsubscribers.forEach(unsub => {
                if (typeof unsub === 'function') unsub();
            });
            this.unsubscribers = [];
        }
        this.voiceService.destroy();
    }
}
