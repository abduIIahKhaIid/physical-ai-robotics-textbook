import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChatProvider } from '../components/ChatWidget/ChatProvider';

export default function Root({ children }) {
  return (
    <ChatProvider>
      {children}
      <BrowserOnly>
        {() => {
          const { ChatWidgetOverlay } = require('../components/ChatWidget');
          return <ChatWidgetOverlay />;
        }}
      </BrowserOnly>
    </ChatProvider>
  );
}
