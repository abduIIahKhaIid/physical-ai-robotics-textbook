import React, { createContext, useContext, useState, useCallback, useRef, useEffect } from 'react';
import { postChat, type SSEEvent, type Citation } from '../../utils/sseClient';
import { getChatApiUrl, SESSION_STORAGE_KEY, MAX_SELECTED_TEXT_LENGTH } from '../../utils/chatConfig';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations: Citation[];
  selectedText: string | null;
  timestamp: number;
}

export interface TextSelection {
  text: string;
  pageUrl: string;
  pageTitle: string;
}

export type UIState = 'empty' | 'idle' | 'loading' | 'streaming' | 'error' | 'rate_limited' | 'offline';

interface ChatContextType {
  isOpen: boolean;
  sessionId: string | null;
  messages: Message[];
  selectedText: TextSelection | null;
  uiState: UIState;
  streamingContent: string;
  errorMessage: string;
  rateLimitSeconds: number;
  togglePanel: () => void;
  sendMessage: (text: string) => void;
  setSelectedText: (selection: TextSelection | null) => void;
  retryLastMessage: () => void;
}

const ChatContext = createContext<ChatContextType | null>(null);

export function useChatContext(): ChatContextType {
  const ctx = useContext(ChatContext);
  if (!ctx) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return ctx;
}

export function ChatProvider({ children }: { children: React.ReactNode }) {
  const [isOpen, setIsOpen] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [messages, setMessages] = useState<Message[]>([]);
  const [selectedText, setSelectedTextState] = useState<TextSelection | null>(null);
  const [uiState, setUiState] = useState<UIState>('empty');
  const [streamingContent, setStreamingContent] = useState('');
  const [errorMessage, setErrorMessage] = useState('');
  const [rateLimitSeconds, setRateLimitSeconds] = useState(0);

  const abortRef = useRef(false);
  const lastUserMessageRef = useRef<string>('');
  const previousUiStateRef = useRef<UIState>('empty');
  const rateLimitTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  // Initialize sessionId from sessionStorage on mount
  useEffect(() => {
    try {
      const stored = sessionStorage.getItem(SESSION_STORAGE_KEY);
      if (stored) {
        setSessionId(stored);
      }
    } catch {
      // sessionStorage unavailable
    }
  }, []);

  // Online/offline detection
  useEffect(() => {
    const handleOnline = () => {
      setUiState(prev => prev === 'offline' ? previousUiStateRef.current : prev);
    };
    const handleOffline = () => {
      setUiState(prev => {
        if (prev !== 'offline') {
          previousUiStateRef.current = prev;
        }
        return 'offline';
      });
    };

    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    if (!navigator.onLine) {
      handleOffline();
    }

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, []);

  // Rate limit countdown
  useEffect(() => {
    if (rateLimitSeconds > 0 && uiState === 'rate_limited') {
      rateLimitTimerRef.current = setInterval(() => {
        setRateLimitSeconds(prev => {
          if (prev <= 1) {
            if (rateLimitTimerRef.current) clearInterval(rateLimitTimerRef.current);
            setUiState('idle');
            return 0;
          }
          return prev - 1;
        });
      }, 1000);
    }
    return () => {
      if (rateLimitTimerRef.current) clearInterval(rateLimitTimerRef.current);
    };
  }, [rateLimitSeconds, uiState]);

  const togglePanel = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  const setSelectedText = useCallback((selection: TextSelection | null) => {
    if (selection && selection.text.length > MAX_SELECTED_TEXT_LENGTH) {
      selection = {
        ...selection,
        text: selection.text.slice(0, MAX_SELECTED_TEXT_LENGTH),
      };
    }
    setSelectedTextState(selection);
  }, []);

  const processStream = useCallback(async (
    messageText: string,
    currentSessionId: string | null,
    currentSelectedText: TextSelection | null,
  ) => {
    abortRef.current = false;
    setStreamingContent('');
    setUiState('loading');
    setErrorMessage('');

    const backendUrl = getChatApiUrl();
    const request = {
      message: messageText,
      session_id: currentSessionId,
      mode: currentSelectedText ? 'selected_text_only' as const : 'normal' as const,
      selected_text: currentSelectedText?.text || null,
      source_doc_path: currentSelectedText?.pageUrl
        ?? (typeof window !== 'undefined' ? window.location.pathname : null),
      source_section: currentSelectedText?.pageTitle
        ?? (typeof document !== 'undefined' ? document.title : null),
    };

    let accumulated = '';
    let finalCitations: Citation[] = [];

    try {
      for await (const event of postChat(backendUrl, request)) {
        if (abortRef.current) break;

        switch (event.type) {
          case 'token':
            accumulated += event.content;
            setStreamingContent(accumulated);
            setUiState('streaming');
            break;

          case 'done': {
            const newSessionId = event.sessionId;
            if (newSessionId) {
              setSessionId(newSessionId);
              try {
                sessionStorage.setItem(SESSION_STORAGE_KEY, newSessionId);
              } catch {
                // sessionStorage unavailable
              }
            }
            finalCitations = event.citations || [];
            const assistantMessage: Message = {
              id: event.messageId || `msg-${Date.now()}`,
              role: 'assistant',
              content: accumulated,
              citations: finalCitations,
              selectedText: null,
              timestamp: Date.now(),
            };
            setMessages(prev => [...prev, assistantMessage]);
            setStreamingContent('');
            setUiState('idle');
            break;
          }

          case 'error':
            setErrorMessage(event.message);
            setUiState('error');
            setStreamingContent('');
            return;

          case 'rate_limited':
            setRateLimitSeconds(event.retryAfter);
            setUiState('rate_limited');
            setStreamingContent('');
            return;

          case 'session_not_found':
            // Clear stale session and retry
            setSessionId(null);
            try {
              sessionStorage.removeItem(SESSION_STORAGE_KEY);
            } catch {
              // sessionStorage unavailable
            }
            console.warn('Session not found. Retrying with new session.');
            // Retry without session_id
            abortRef.current = true;
            setTimeout(() => {
              processStream(messageText, null, currentSelectedText);
            }, 0);
            return;
        }
      }
    } catch {
      setErrorMessage('Connection interrupted. Please try again.');
      setUiState('error');
      setStreamingContent('');
    }
  }, []);

  const sendMessage = useCallback((text: string) => {
    const trimmed = text.trim();
    if (!trimmed) return;

    lastUserMessageRef.current = trimmed;

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: trimmed,
      citations: [],
      selectedText: selectedText?.text || null,
      timestamp: Date.now(),
    };

    setMessages(prev => [...prev, userMessage]);
    processStream(trimmed, sessionId, selectedText);
  }, [sessionId, selectedText, processStream]);

  const retryLastMessage = useCallback(() => {
    if (!lastUserMessageRef.current) return;
    // Remove the last error/assistant message and retry
    setMessages(prev => {
      // Keep the last user message, remove any trailing error state
      return prev;
    });
    processStream(lastUserMessageRef.current, sessionId, selectedText);
  }, [sessionId, selectedText, processStream]);

  const value: ChatContextType = {
    isOpen,
    sessionId,
    messages,
    selectedText,
    uiState,
    streamingContent,
    errorMessage,
    rateLimitSeconds,
    togglePanel,
    sendMessage,
    setSelectedText,
    retryLastMessage,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
}
