import React, { useRef, useEffect, useState, useCallback } from 'react';
import { useChatContext } from './ChatProvider';
import { MessageBubble } from './MessageBubble';
import { WelcomeScreen } from './WelcomeScreen';
import { SelectedTextChip } from './SelectedTextChip';

export function ChatPanel() {
  const {
    isOpen,
    messages,
    uiState,
    streamingContent,
    selectedText,
    errorMessage,
    rateLimitSeconds,
    togglePanel,
    sendMessage,
    retryLastMessage,
  } = useChatContext();

  const [inputValue, setInputValue] = useState('');
  const [isMobile, setIsMobile] = useState(false);
  const messageListRef = useRef<HTMLDivElement>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const isNearBottomRef = useRef(true);

  // Detect mobile viewport
  useEffect(() => {
    const mq = window.matchMedia('(max-width: 767px)');
    setIsMobile(mq.matches);
    const handler = (e: MediaQueryListEvent) => setIsMobile(e.matches);
    mq.addEventListener('change', handler);
    return () => mq.removeEventListener('change', handler);
  }, []);

  // Focus textarea when panel opens
  useEffect(() => {
    if (isOpen && textareaRef.current) {
      textareaRef.current.focus();
    }
  }, [isOpen]);

  // Auto-scroll: only if user is near bottom
  useEffect(() => {
    const el = messageListRef.current;
    if (el && isNearBottomRef.current) {
      el.scrollTop = el.scrollHeight;
    }
  }, [messages.length, streamingContent]);

  // Track scroll position
  const handleScroll = useCallback(() => {
    const el = messageListRef.current;
    if (el) {
      isNearBottomRef.current = el.scrollHeight - el.scrollTop - el.clientHeight < 50;
    }
  }, []);

  // Escape key to close
  useEffect(() => {
    if (!isOpen) return;
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        togglePanel();
      }
    };
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, togglePanel]);

  if (!isOpen) return null;

  const handleSend = () => {
    const text = inputValue.trim();
    if (!text) return;
    if (uiState === 'loading' || uiState === 'streaming') return;
    setInputValue('');
    sendMessage(text);
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleTextareaChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setInputValue(e.target.value);
    // Auto-grow
    const ta = e.target;
    ta.style.height = 'auto';
    ta.style.height = Math.min(ta.scrollHeight, 100) + 'px';
  };

  const isInputDisabled = uiState === 'loading' || uiState === 'streaming' || uiState === 'offline';

  return (
    <>
      {/* Mobile backdrop */}
      {isMobile && (
        <div
          className="chat-panel-backdrop"
          onClick={togglePanel}
          aria-hidden="true"
        />
      )}

      <div
        className={`chat-panel${isMobile ? ' chat-panel-mobile' : ''}`}
        role="dialog"
        aria-modal="true"
        aria-label="RoboTutor Chat"
      >
        {/* Header */}
        <div className="chat-header">
          <div className="chat-header-title">
            <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M12 8V4H8" />
              <rect width="16" height="12" x="4" y="8" rx="2" />
              <path d="m2 14 6-6 6 6" />
            </svg>
            RoboTutor
            {selectedText && (
              <span className="selection-mode-badge">Selection mode</span>
            )}
          </div>
          <button
            className="chat-header-close"
            onClick={togglePanel}
            aria-label="Close chat"
          >
            <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>

        {/* Offline banner */}
        {uiState === 'offline' && (
          <div className="chat-offline-banner">
            You&apos;re offline. Reconnect to send messages.
          </div>
        )}

        {/* Message area */}
        <div
          className="message-list"
          ref={messageListRef}
          onScroll={handleScroll}
          role="log"
          aria-live="polite"
        >
          {uiState === 'empty' && messages.length === 0 ? (
            <WelcomeScreen />
          ) : (
            <>
              {messages.map((msg) => (
                <MessageBubble key={msg.id} message={msg} />
              ))}

              {/* Streaming message */}
              {uiState === 'streaming' && streamingContent && (
                <div className="message-bubble message-bubble-assistant">
                  {streamingContent}
                </div>
              )}

              {/* Loading indicator */}
              {uiState === 'loading' && (
                <div className="typing-indicator">
                  <span />
                  <span />
                  <span />
                </div>
              )}

              {/* Error state */}
              {uiState === 'error' && (
                <div className="chat-error-bubble">
                  <span>{errorMessage || 'Something went wrong.'}</span>
                  <button className="chat-error-retry" onClick={retryLastMessage}>
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                      <polyline points="23 4 23 10 17 10" />
                      <path d="M20.49 15a9 9 0 1 1-2.12-9.36L23 10" />
                    </svg>
                    Retry
                  </button>
                </div>
              )}

              {/* Rate limited */}
              {uiState === 'rate_limited' && (
                <div className="chat-rate-limit">
                  Too many requests. Try again in {rateLimitSeconds}s.
                </div>
              )}
            </>
          )}
        </div>

        {/* Selected text chip */}
        {selectedText && <SelectedTextChip />}

        {/* Input area */}
        <div className="chat-input-area">
          <textarea
            ref={textareaRef}
            className="chat-textarea"
            value={inputValue}
            onChange={handleTextareaChange}
            onKeyDown={handleKeyDown}
            placeholder="Ask a question..."
            aria-label="Type your message"
            disabled={isInputDisabled}
            rows={1}
          />
          <button
            className="chat-send-button"
            onClick={handleSend}
            disabled={isInputDisabled || !inputValue.trim()}
            aria-label="Send message"
          >
            <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <line x1="22" y1="2" x2="11" y2="13" />
              <polygon points="22 2 15 22 11 13 2 9 22 2" />
            </svg>
          </button>
        </div>
      </div>
    </>
  );
}
