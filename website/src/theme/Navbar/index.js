import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';
import Navbar from '@theme-original/Navbar';
import BrowserOnly from '@docusaurus/BrowserOnly';

const AUTH_NEXT_URL = 'https://auth-next-zeta.vercel.app';
const TOKEN_KEY = 'bearer_token';
const USER_KEY = 'auth_user_name';

function AuthButtons() {
  const [portalTarget, setPortalTarget] = useState(null);
  const [user, setUser] = useState(null); // { name }
  const [dropdownOpen, setDropdownOpen] = useState(false);

  // On mount: check URL params for token handoff, then check localStorage
  useEffect(() => {
    // Capture token from redirect
    const params = new URLSearchParams(window.location.search);
    const token = params.get('token');
    const name = params.get('name');
    if (token) {
      localStorage.setItem(TOKEN_KEY, token);
      if (name) localStorage.setItem(USER_KEY, name);
      // Clean URL without reload
      const url = new URL(window.location);
      url.searchParams.delete('token');
      url.searchParams.delete('name');
      window.history.replaceState({}, '', url.pathname + url.hash);
    }

    // Check if user is logged in
    const storedToken = localStorage.getItem(TOKEN_KEY);
    const storedName = localStorage.getItem(USER_KEY);
    if (storedToken) {
      setUser({ name: storedName || 'User' });
    }
  }, []);

  // Close dropdown on outside click
  useEffect(() => {
    if (!dropdownOpen) return;
    function handleClick(e) {
      if (!e.target.closest('.navbar-auth-buttons')) {
        setDropdownOpen(false);
      }
    }
    document.addEventListener('click', handleClick);
    return () => document.removeEventListener('click', handleClick);
  }, [dropdownOpen]);

  // Find navbar right-items container
  useEffect(() => {
    const rightItems = document.querySelector('.navbar__items--right');
    if (!rightItems) return;

    let container = rightItems.querySelector('.navbar-auth-portal');
    if (!container) {
      container = document.createElement('div');
      container.className = 'navbar-auth-portal';

      const toggle = Array.from(rightItems.children).find(
        (el) => el.className && /colorModeToggle/.test(el.className)
      );
      if (toggle) {
        rightItems.insertBefore(container, toggle);
      } else {
        rightItems.appendChild(container);
      }
    }
    setPortalTarget(container);

    return () => {
      if (container && container.parentNode) {
        container.parentNode.removeChild(container);
      }
    };
  }, []);

  if (!portalTarget) return null;

  function handleSignOut() {
    localStorage.removeItem(TOKEN_KEY);
    localStorage.removeItem(USER_KEY);
    setUser(null);
    setDropdownOpen(false);
  }

  const initials = user?.name
    ? user.name.split(' ').map(w => w[0]).join('').toUpperCase().slice(0, 2)
    : 'U';

  const content = user ? (
    <div className="navbar-auth-buttons">
      <button
        className="navbar-auth-user-btn"
        onClick={() => setDropdownOpen(!dropdownOpen)}
        type="button"
      >
        <span className="navbar-auth-avatar">{initials}</span>
        <span className="navbar-auth-username">{user.name}</span>
        <svg width="12" height="12" viewBox="0 0 24 24" fill="none"
          stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"
          style={{ opacity: 0.5, transition: 'transform 0.2s', transform: dropdownOpen ? 'rotate(180deg)' : 'none' }}>
          <path d="M6 9l6 6 6-6" />
        </svg>
      </button>
      {dropdownOpen && (
        <div className="navbar-auth-dropdown">
          <div className="navbar-auth-dropdown-header">
            <strong>{user.name}</strong>
          </div>
          <hr className="navbar-auth-dropdown-divider" />
          <button className="navbar-auth-dropdown-item" onClick={handleSignOut} type="button">
            Sign out
          </button>
        </div>
      )}
    </div>
  ) : (
    <div className="navbar-auth-buttons">
      <a className="navbar-auth-signin-btn" href={`${AUTH_NEXT_URL}/login`}>
        Sign In
      </a>
      <a className="navbar-auth-getstarted-btn" href={`${AUTH_NEXT_URL}/register`}>
        Get Started
      </a>
    </div>
  );

  return ReactDOM.createPortal(content, portalTarget);
}

export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props} />
      <BrowserOnly>{() => <AuthButtons />}</BrowserOnly>
    </>
  );
}
