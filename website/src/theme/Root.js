import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChatProvider } from '../components/ChatWidget/ChatProvider';

export default function Root({ children }) {
  return (
    <BrowserOnly fallback={<>{children}</>}>
      {() => {
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
      }}
    </BrowserOnly>
  );
}
