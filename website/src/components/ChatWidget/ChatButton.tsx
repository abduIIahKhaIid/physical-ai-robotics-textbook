import React from 'react';
import { useChatContext } from './ChatProvider';

export function ChatButton() {
  const { isOpen, togglePanel } = useChatContext();

  if (isOpen) return null;

  return (
    <button
      className="chat-button"
      onClick={togglePanel}
      aria-label="Open RoboTutor chat"
      aria-expanded={false}
      tabIndex={0}
      onKeyDown={(e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          togglePanel();
        }
      }}
    >
      <svg
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
      </svg>
    </button>
  );
}
