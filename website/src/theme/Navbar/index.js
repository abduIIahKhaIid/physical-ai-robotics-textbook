import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';
import Navbar from '@theme-original/Navbar';
import BrowserOnly from '@docusaurus/BrowserOnly';

const AUTH_NEXT_URL = 'https://auth-next-zeta.vercel.app';

function AuthButtons() {
  const [portalTarget, setPortalTarget] = useState(null);

  // Find the navbar right-items container and insert auth buttons before the toggle
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

  const content = (
    <div className="navbar-auth-buttons">
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
