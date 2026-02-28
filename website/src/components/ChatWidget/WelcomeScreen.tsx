import React from 'react';
import { useChatContext } from './ChatProvider';

const AUTH_NEXT_URL = 'https://auth-next-zeta.vercel.app';

const SUGGESTIONS = [
  'What is sim-to-real transfer?',
  'Explain inverse kinematics',
  'How does SLAM work?',
];

function AuthSection() {
  return (
    <div className="welcome-auth-section">
      <a
        className="welcome-auth-btn"
        href={`${AUTH_NEXT_URL}/login`}
        target="_blank"
        rel="noopener noreferrer"
      >
        Sign In
      </a>
      <a
        className="welcome-auth-btn welcome-auth-btn-primary"
        href={`${AUTH_NEXT_URL}/register`}
        target="_blank"
        rel="noopener noreferrer"
      >
        Sign Up
      </a>
    </div>
  );
}

export function WelcomeScreen() {
  const { sendMessage } = useChatContext();

  return (
    <div className="welcome-screen">
      <svg
        className="welcome-icon"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <path d="M12 8V4H8" />
        <rect width="16" height="12" x="4" y="8" rx="2" />
        <path d="m2 14 6-6 6 6" />
        <path d="M9.5 16a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0Z" />
        <path d="M15.5 16a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0Z" />
      </svg>
      <div className="welcome-title">Ask RoboTutor</div>
      <div className="welcome-subtitle">
        Ask me about any topic in the textbook
      </div>
      <AuthSection />
      <div className="welcome-chips">
        {SUGGESTIONS.map((text) => (
          <button
            key={text}
            className="welcome-chip"
            onClick={() => sendMessage(text)}
          >
            {text}
          </button>
        ))}
      </div>
    </div>
  );
}
