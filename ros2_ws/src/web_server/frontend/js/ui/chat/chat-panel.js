/**
 * ReloBot AI Chat Panel Controller
 * Coordinates chat UI, message rendering, live voice transcription, and token-by-token streaming.
 */

import { ChatService } from '../../services/chat-service.js';
import { VoiceRecognitionService } from '../../services/voice-recognition-service.js';

export class ChatPanel {
    constructor() {
        this.chatService = new ChatService();
        this.voiceService = new VoiceRecognitionService();

        // DOM elements
        this.feed = document.getElementById('chatFeed');
        this.input = document.getElementById('chatInputText');
        this.sendBtn = document.getElementById('chatSendBtn');
        this.micBtn = document.getElementById('chatMicBtn');
        this.liveBox = document.getElementById('chatLiveBox');
        this.liveText = document.getElementById('chatLiveText');
        this.waveform = document.getElementById('chatWaveform');
        this.statusBadge = document.getElementById('chatStatusBadge');
        this.statusDot = document.getElementById('chatStatusDot');
        this.toggleRobotSpeakerBtn = document.getElementById('toggleRobotSpeaker');
        this.toggleBrowserAudioBtn = document.getElementById('toggleBrowserAudio');
        this.clearBtn = document.getElementById('chatClearBtn');
        this.chips = document.querySelectorAll('.c-chat-chip');

        // State
        this.currentBotMsgBubble = null;
        this.currentBotMsgText = '';
        this.activeMsgId = null;
        this.isGenerating = false;
    }

    init() {
        this.chatService.init();
        this._setupChatServiceEvents();
        this._setupVoiceEvents();
        this._setupDomListeners();
        this._updateAudioToggleStates();
    }

    _setupChatServiceEvents() {
        this.chatService.onConnectionChange = (connected) => {
            if (this.statusBadge && this.statusDot) {
                if (connected) {
                    this.statusDot.style.backgroundColor = 'var(--color-green)';
                    this.statusDot.style.boxShadow = '0 0 6px var(--color-green-glow)';
                    this.statusBadge.textContent = 'Online';
                } else {
                    this.statusDot.style.backgroundColor = 'var(--color-red)';
                    this.statusDot.style.boxShadow = '0 0 6px var(--color-red-glow)';
                    this.statusBadge.textContent = 'Disconnected';
                }
            }
        };

        this.chatService.onStart = (msgId) => {
            this.isGenerating = true;
            this.activeMsgId = msgId;
            if (this.statusBadge) this.statusBadge.textContent = 'Thinking...';
            this._createBotMessageBubble(msgId);
        };

        this.chatService.onToken = (token, msgId) => {
            if (this.currentBotMsgBubble && this.activeMsgId === msgId) {
                if (this.statusBadge) this.statusBadge.textContent = 'Streaming...';
                this.currentBotMsgText += token;
                this._updateBotMessageBubble(this.currentBotMsgText, false);
                this._scrollToBottom();
            }
        };

        this.chatService.onDone = (fullText, msgId) => {
            this.isGenerating = false;
            if (this.statusBadge) this.statusBadge.textContent = 'Online';
            if (this.currentBotMsgBubble && this.activeMsgId === msgId) {
                this.currentBotMsgText = fullText || this.currentBotMsgText;
                this._updateBotMessageBubble(this.currentBotMsgText, true);
                this._scrollToBottom();
            }
            this.currentBotMsgBubble = null;
            this.currentBotMsgText = '';
            this.activeMsgId = null;
        };

        this.chatService.onError = (errorMsg) => {
            this.isGenerating = false;
            if (this.statusBadge) this.statusBadge.textContent = 'Error';
            if (this.currentBotMsgBubble) {
                this._updateBotMessageBubble(`${this.currentBotMsgText}\n\n*[Error: ${errorMsg}]*`, true);
            }
            this.currentBotMsgBubble = null;
            this.activeMsgId = null;
        };

        this.chatService.onAudioPlaying = (isPlaying) => {
            if (this.statusBadge && !this.isGenerating) {
                this.statusBadge.textContent = isPlaying ? 'Speaking...' : 'Online';
            }
        };
    }

    _setupVoiceEvents() {
        this.voiceService.onStart = () => {
            if (this.micBtn) this.micBtn.classList.add('is-recording');
            if (this.liveBox) this.liveBox.classList.add('is-visible');
            if (this.waveform) this.waveform.classList.add('is-animating');
            if (this.liveText) this.liveText.textContent = 'Listening...';
        };

        this.voiceService.onInterimResult = (transcript) => {
            if (this.liveText) {
                this.liveText.textContent = `"${transcript}"`;
            }
            if (this.input) {
                this.input.value = transcript;
                this._autoGrowInput();
            }
            this._updateSendButtonState();
        };

        this.voiceService.onFinalResult = (transcript) => {
            if (this.liveText) {
                this.liveText.textContent = `"${transcript}"`;
            }
            if (this.input) {
                this.input.value = transcript;
                this._autoGrowInput();
            }
            this._updateSendButtonState();

            // Auto-send on speech completion for seamless hands-free interaction
            if (transcript.trim()) {
                setTimeout(() => {
                    this.voiceService.stopRecording();
                    this._sendMessage(transcript.trim());
                }, 400);
            }
        };

        this.voiceService.onAudioVolume = (vol) => {
            if (!this.waveform) return;
            const bars = this.waveform.querySelectorAll('.c-chat-waveform__bar');
            bars.forEach((bar, idx) => {
                const scale = Math.max(0.2, (vol * (1 + (idx % 3) * 0.4)));
                bar.style.height = `${Math.min(18, Math.max(4, scale * 18))}px`;
            });
        };

        this.voiceService.onEnd = () => {
            if (this.micBtn) this.micBtn.classList.remove('is-recording');
            if (this.liveBox) this.liveBox.classList.remove('is-visible');
            if (this.waveform) this.waveform.classList.remove('is-animating');
        };

        this.voiceService.onError = (err) => {
            console.warn('[ChatPanel] Voice recognition error:', err);
            if (this.liveText) this.liveText.textContent = `Microphone error: ${err}`;
            setTimeout(() => {
                if (this.liveBox) this.liveBox.classList.remove('is-visible');
            }, 2500);
        };
    }

    _setupDomListeners() {
        // Mic button click
        if (this.micBtn) {
            this.micBtn.addEventListener('click', () => {
                if (this.voiceService.isRecording) {
                    this.voiceService.stopRecording();
                } else {
                    this.voiceService.startRecording();
                }
            });
        }

        // Send button click
        if (this.sendBtn) {
            this.sendBtn.addEventListener('click', () => {
                const text = this.input ? this.input.value.trim() : '';
                if (text) {
                    this._sendMessage(text);
                }
            });
        }

        // Textarea input and keydown
        if (this.input) {
            this.input.addEventListener('input', () => {
                this._autoGrowInput();
                this._updateSendButtonState();
            });

            this.input.addEventListener('keydown', (e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault();
                    const text = this.input.value.trim();
                    if (text) {
                        this._sendMessage(text);
                    }
                }
            });
        }

        // Suggestion chips
        if (this.chips) {
            this.chips.forEach(chip => {
                chip.addEventListener('click', () => {
                    const prompt = chip.getAttribute('data-prompt') || chip.textContent.trim();
                    if (prompt) {
                        this._sendMessage(prompt);
                    }
                });
            });
        }

        // Audio toggles
        if (this.toggleRobotSpeakerBtn) {
            this.toggleRobotSpeakerBtn.addEventListener('click', () => {
                const newState = !this.chatService.playRobotAudio;
                this.chatService.setPlayRobotAudio(newState);
                this._updateAudioToggleStates();
            });
        }

        if (this.toggleBrowserAudioBtn) {
            this.toggleBrowserAudioBtn.addEventListener('click', () => {
                const newState = !this.chatService.playBrowserAudio;
                this.chatService.setPlayBrowserAudio(newState);
                this._updateAudioToggleStates();
            });
        }

        // Clear chat
        if (this.clearBtn) {
            this.clearBtn.addEventListener('click', () => {
                this._clearChat();
            });
        }
    }

    _sendMessage(text) {
        if (!text) return;

        // Render user message bubble
        this._createUserMessageBubble(text);

        // Reset input
        if (this.input) {
            this.input.value = '';
            this.input.style.height = 'auto';
        }
        this._updateSendButtonState();
        this._scrollToBottom();

        // Send to chat service
        this.chatService.sendPrompt(text);
    }

    _createUserMessageBubble(text) {
        if (!this.feed) return;
        const msgDiv = document.createElement('div');
        msgDiv.className = 'c-chat-msg c-chat-msg--user';

        const timeStr = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

        msgDiv.innerHTML = `
            <div class="c-chat-msg__avatar"><i class="fas fa-user"></i></div>
            <div class="c-chat-msg__content">
                <div class="c-chat-msg__bubble">${this._escapeHtml(text)}</div>
                <div class="c-chat-msg__time">${timeStr}</div>
            </div>
        `;
        this.feed.appendChild(msgDiv);
    }

    _createBotMessageBubble(msgId) {
        if (!this.feed) return;
        const msgDiv = document.createElement('div');
        msgDiv.className = 'c-chat-msg c-chat-msg--bot';
        msgDiv.id = `bot_${msgId}`;

        const timeStr = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

        msgDiv.innerHTML = `
            <div class="c-chat-msg__avatar"><i class="fas fa-robot"></i></div>
            <div class="c-chat-msg__content">
                <div class="c-chat-msg__bubble" id="bubble_${msgId}">
                    <span class="c-chat-cursor"></span>
                </div>
                <div class="c-chat-msg__time">
                    <span>${timeStr}</span>
                    <button class="c-chat-msg__replay-btn" id="replay_${msgId}" style="display: none;" title="Speak aloud">
                        <i class="fas fa-volume-high"></i> Replay
                    </button>
                </div>
            </div>
        `;
        this.feed.appendChild(msgDiv);
        this.currentBotMsgBubble = document.getElementById(`bubble_${msgId}`);
        this.currentBotMsgText = '';
        this._scrollToBottom();
    }

    _updateBotMessageBubble(rawText, isFinal = false) {
        if (!this.currentBotMsgBubble) return;

        const rendered = this._renderMarkdown(rawText);
        if (isFinal) {
            this.currentBotMsgBubble.innerHTML = rendered;
            const parentMsg = this.currentBotMsgBubble.closest('.c-chat-msg');
            if (parentMsg) {
                const replayBtn = parentMsg.querySelector('.c-chat-msg__replay-btn');
                if (replayBtn) {
                    replayBtn.style.display = 'inline-flex';
                    replayBtn.onclick = () => {
                        this.chatService.sendPrompt(`Repeat: ${rawText}`);
                    };
                }
            }
        } else {
            this.currentBotMsgBubble.innerHTML = `${rendered}<span class="c-chat-cursor"></span>`;
        }
    }

    _renderMarkdown(text) {
        if (!text) return '';
        let html = this._escapeHtml(text);

        // Bold **text**
        html = html.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
        // Italic *text*
        html = html.replace(/\*([^*]+)\*/g, '<em>$1</em>');
        // Inline code `code`
        html = html.replace(/`([^`]+)`/g, '<code style="background: rgba(0,0,0,0.3); padding: 2px 4px; border-radius: 4px; font-family: monospace; color: var(--color-green);">$1</code>');
        // Line breaks
        html = html.replace(/\n/g, '<br>');

        return html;
    }

    _escapeHtml(str) {
        const div = document.createElement('div');
        div.textContent = str;
        return div.innerHTML;
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

    _clearChat() {
        if (!this.feed) return;
        const welcome = this.feed.querySelector('.c-chat-welcome');
        this.feed.innerHTML = '';
        if (welcome) {
            this.feed.appendChild(welcome);
        }
    }

    destroy() {
        this.voiceService.destroy();
        this.chatService.destroy();
    }
}
