import React from 'react';
import { ChatButton } from './ChatButton';
import { ChatPanel } from './ChatPanel';
import { SelectionTooltip } from './SelectionTooltip';
import './ChatWidget.css';

export function ChatWidgetOverlay() {
  return (
    <>
      <ChatButton />
      <ChatPanel />
      <SelectionTooltip />
    </>
  );
}

/**
 * Post-Deploy Verification Checklist (GitHub Pages)
 *
 * 1. Chat button visible on deployed docs URL
 *    → https://abduIIahKhaIid.github.io/physical-ai-robotics-textbook/docs/
 *
 * 2. Panel opens on click
 *    → Click FAB → panel appears with welcome screen
 *
 * 3. Backend CORS allows GitHub Pages origin
 *    → Verify CORS_ORIGINS includes "https://abduIIahKhaIid.github.io"
 *    → No blocked cross-origin requests in console
 *
 * 4. SSE streaming works over HTTPS
 *    → Send a message → tokens stream in → done event fires
 *    → Network tab shows text/event-stream response
 *
 * 5. No mixed-content warnings
 *    → All requests to backend use HTTPS (not HTTP)
 *    → Console has no mixed-content errors
 */
