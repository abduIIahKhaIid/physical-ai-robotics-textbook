import React, { useEffect, useState, useCallback } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import useBaseUrl from '@docusaurus/useBaseUrl';

/**
 * ChatKit Widget for Docusaurus
 * Supports both normal and selection-only modes
 * Handles token-based auth and session management
 */
const ChatKitWidget = ({ 
  mode = 'normal', 
  authType = 'none',
  backendUrl = 'http://localhost:8000'
}) => {
  const [isLoaded, setIsLoaded] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Initialize session
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    let sid = localStorage.getItem('chatkit_session_id');
    if (!sid) {
      sid = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chatkit_session_id', sid);
    }
    setSessionId(sid);
    setIsLoaded(true);
  }, []);

  // Selection capture for selection-only mode
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM || mode !== 'selection_only') return;

    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      if (text.length > 0 && text.length < 12000) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [mode]);

  // Fetch token if using auth
  const fetchToken = useCallback(async () => {
    if (authType === 'none') return null;
    
    try {
      const response = await fetch(`${backendUrl}/auth/token`, {
        credentials: 'include'
      });
      const data = await response.json();
      return data.token;
    } catch (error) {
      console.error('Failed to fetch token:', error);
      return null;
    }
  }, [authType, backendUrl]);

  // Send message to backend
  const sendMessage = useCallback(async (text) => {
    if (!text.trim() || !sessionId) return;

    setIsLoading(true);
    
    try {
      const token = await fetchToken();
      
      const payload = {
        message: text,
        session_id: sessionId,
        mode: mode,
        ...(mode === 'selection_only' && selectedText && { selected_text: selectedText })
      };

      const headers = {
        'Content-Type': 'application/json',
        ...(token && { 'Authorization': `Bearer ${token}` })
      };

      const response = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers,
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }

      const data = await response.json();
      
      setMessages(prev => [
        ...prev,
        { role: 'user', content: text },
        { role: 'assistant', content: data.response }
      ]);
      
      setInputValue('');
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [
        ...prev,
        { role: 'user', content: text },
        { role: 'assistant', content: 'Sorry, I encountered an error. Please try again.' }
      ]);
    } finally {
      setIsLoading(false);
    }
  }, [sessionId, mode, selectedText, backendUrl, fetchToken]);

  // Handle form submit
  const handleSubmit = (e) => {
    e.preventDefault();
    sendMessage(inputValue);
  };

  if (!ExecutionEnvironment.canUseDOM || !isLoaded) {
    return null;
  }

  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      zIndex: 1000
    }}>
      {/* Toggle Button */}
      {!isOpen && (
        <button
          onClick={() => setIsOpen(true)}
          style={{
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#007bff',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            cursor: 'pointer',
            boxShadow: '0 4px 8px rgba(0,0,0,0.2)'
          }}
        >
          💬
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div style={{
          width: '350px',
          height: '500px',
          backgroundColor: 'white',
          borderRadius: '10px',
          boxShadow: '0 4px 16px rgba(0,0,0,0.3)',
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden'
        }}>
          {/* Header */}
          <div style={{
            padding: '15px',
            backgroundColor: '#007bff',
            color: 'white',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center'
          }}>
            <h3 style={{ margin: 0, fontSize: '16px' }}>
              Chat Assistant
              {mode === 'selection_only' && selectedText && ' (Selection Mode)'}
            </h3>
            <button
              onClick={() => setIsOpen(false)}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '20px',
                cursor: 'pointer'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages */}
          <div style={{
            flex: 1,
            padding: '15px',
            overflowY: 'auto',
            backgroundColor: '#f5f5f5'
          }}>
            {messages.length === 0 && (
              <div style={{ color: '#999', textAlign: 'center', marginTop: '50px' }}>
                {mode === 'selection_only' 
                  ? 'Select text on the page and ask questions about it'
                  : 'Start a conversation...'}
              </div>
            )}
            {messages.map((msg, idx) => (
              <div
                key={idx}
                style={{
                  marginBottom: '10px',
                  textAlign: msg.role === 'user' ? 'right' : 'left'
                }}
              >
                <div style={{
                  display: 'inline-block',
                  padding: '8px 12px',
                  borderRadius: '8px',
                  maxWidth: '80%',
                  backgroundColor: msg.role === 'user' ? '#007bff' : 'white',
                  color: msg.role === 'user' ? 'white' : 'black',
                  boxShadow: '0 1px 2px rgba(0,0,0,0.1)'
                }}>
                  {msg.content}
                </div>
              </div>
            ))}
            {isLoading && (
              <div style={{ textAlign: 'left', color: '#999' }}>
                Typing...
              </div>
            )}
          </div>

          {/* Input */}
          <form onSubmit={handleSubmit} style={{
            padding: '15px',
            borderTop: '1px solid #ddd',
            display: 'flex',
            gap: '10px'
          }}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder={
                mode === 'selection_only' && !selectedText
                  ? 'Select text first...'
                  : 'Type your message...'
              }
              disabled={isLoading || (mode === 'selection_only' && !selectedText)}
              style={{
                flex: 1,
                padding: '8px 12px',
                border: '1px solid #ddd',
                borderRadius: '20px',
                outline: 'none'
              }}
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading || (mode === 'selection_only' && !selectedText)}
              style={{
                padding: '8px 16px',
                backgroundColor: '#007bff',
                color: 'white',
                border: 'none',
                borderRadius: '20px',
                cursor: 'pointer',
                opacity: (!inputValue.trim() || isLoading) ? 0.5 : 1
              }}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatKitWidget;