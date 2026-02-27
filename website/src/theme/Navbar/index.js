import React, { useState, useRef, useEffect } from 'react';
import ReactDOM from 'react-dom';
import Navbar from '@theme-original/Navbar';
import BrowserOnly from '@docusaurus/BrowserOnly';

const AUTH_NEXT_URL = 'https://auth-next-zeta.vercel.app';

function AuthButtons() {
  const { useAuth } = require('../../components/AuthProvider');

  const { user, isAuthenticated, isLoading, signOut } = useAuth();
  const [showDropdown, setShowDropdown] = useState(null);
  const dropdownRef = useRef(null);
  const [portalTarget, setPortalTarget] = useState(null);

  // Find the navbar right-items container and insert auth buttons before the toggle
  useEffect(() => {
    const rightItems = document.querySelector('.navbar__items--right');
    if (!rightItems) return;

    // Create a container for auth buttons if it doesn't exist
    let container = rightItems.querySelector('.navbar-auth-portal');
    if (!container) {
      container = document.createElement('div');
      container.className = 'navbar-auth-portal';

      // Insert before the color mode toggle (identified by partial class match)
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
      // Cleanup on unmount
      if (container && container.parentNode) {
        container.parentNode.removeChild(container);
      }
    };
  }, []);

  // Close dropdown on outside click
  useEffect(() => {
    function handleClickOutside(e) {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target)) {
        setShowDropdown(null);
      }
    }
    if (showDropdown) {
      document.addEventListener('mousedown', handleClickOutside);
    }
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [showDropdown]);

  // Close dropdown on Escape
  useEffect(() => {
    function handleEscape(e) {
      if (e.key === 'Escape') setShowDropdown(null);
    }
    if (showDropdown) {
      document.addEventListener('keydown', handleEscape);
    }
    return () => document.removeEventListener('keydown', handleEscape);
  }, [showDropdown]);

  if (isLoading || !portalTarget) return null;

  const content = isAuthenticated ? (
    <div className="navbar-auth-buttons" ref={dropdownRef}>
      <button
        className="navbar-auth-user-btn"
        onClick={() => setShowDropdown(showDropdown === 'user' ? null : 'user')}
        aria-expanded={showDropdown === 'user'}
      >
        <span className="navbar-auth-avatar">
          {user.name?.charAt(0)?.toUpperCase() || 'U'}
        </span>
        <span className="navbar-auth-username">{user.name}</span>
      </button>
      {showDropdown === 'user' && (
        <div className="navbar-auth-dropdown">
          <div className="navbar-auth-dropdown-header">
            <strong>{user.name}</strong>
            <span className="navbar-auth-dropdown-email">{user.email}</span>
          </div>
          <hr className="navbar-auth-dropdown-divider" />
          <button
            className="navbar-auth-dropdown-item"
            onClick={() => { signOut(); setShowDropdown(null); }}
          >
            Sign Out
          </button>
        </div>
      )}
    </div>
  ) : (
    <div className="navbar-auth-buttons" ref={dropdownRef}>
      <a
        className="navbar-auth-signin-btn"
        href={`${AUTH_NEXT_URL}/login`}
      >
        Sign In
      </a>
      <a
        className="navbar-auth-getstarted-btn"
        href={`${AUTH_NEXT_URL}/register`}
      >
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
