import React, { useState } from 'react';
import { useChatContext } from './ChatProvider';

const SUGGESTIONS = [
  'What is sim-to-real transfer?',
  'Explain inverse kinematics',
  'How does SLAM work?',
];

// Auth components loaded conditionally to avoid SSR issues
function AuthSection() {
  const [authView, setAuthView] = useState<'none' | 'login' | 'register'>('none');

  // Lazy-load auth components (they use browser-only APIs)
  let authUser: any = null;
  let isAuthenticated = false;
  let signOut: (() => Promise<void>) | null = null;
  let onboardingCompleted: boolean | null = null;

  try {
    const { useAuth } = require('../AuthProvider');
    const auth = useAuth();
    authUser = auth.user;
    isAuthenticated = auth.isAuthenticated;
    signOut = auth.signOut;
    onboardingCompleted = auth.onboardingCompleted;
  } catch {
    // AuthProvider not available
  }

  if (isAuthenticated && authUser) {
    return (
      <div className="welcome-auth-section">
        <span className="welcome-user-email">{authUser.email}</span>
        {signOut && (
          <button
            className="welcome-auth-btn welcome-signout-btn"
            onClick={() => signOut!()}
          >
            Sign Out
          </button>
        )}
      </div>
    );
  }

  if (authView === 'login') {
    const { LoginForm } = require('../LoginForm');
    return (
      <LoginForm
        onSuccess={() => setAuthView('none')}
        onSwitchToRegister={() => setAuthView('register')}
      />
    );
  }

  if (authView === 'register') {
    const { RegisterForm } = require('../RegisterForm');
    return (
      <RegisterForm
        onSuccess={() => setAuthView('none')}
        onSwitchToLogin={() => setAuthView('login')}
      />
    );
  }

  return (
    <div className="welcome-auth-section">
      <button
        className="welcome-auth-btn"
        onClick={() => setAuthView('login')}
      >
        Sign In
      </button>
      <button
        className="welcome-auth-btn welcome-auth-btn-primary"
        onClick={() => setAuthView('register')}
      >
        Sign Up
      </button>
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
