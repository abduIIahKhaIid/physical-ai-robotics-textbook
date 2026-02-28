import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChatProvider } from '../components/ChatWidget/ChatProvider';

function BrowserRoot({ children }) {
  try {
    const { AuthProvider } = require('../components/AuthProvider');
    const { ChatWidgetOverlay } = require('../components/ChatWidget');
    return (
      <AuthProvider>
        <ChatProvider>
          {children}
          <ChatWidgetOverlay />
        </ChatProvider>
      </AuthProvider>
    );
  } catch (e) {
    // Auth module not available — render without auth
    console.warn('AuthProvider failed to load, running without auth:', e.message);
    const { ChatWidgetOverlay } = require('../components/ChatWidget');
    return (
      <ChatProvider>
        {children}
        <ChatWidgetOverlay />
      </ChatProvider>
    );
  }
}

export default function Root({ children }) {
  return (
    <BrowserOnly fallback={<>{children}</>}>
      {() => <BrowserRoot>{children}</BrowserRoot>}
    </BrowserOnly>
  );
}
