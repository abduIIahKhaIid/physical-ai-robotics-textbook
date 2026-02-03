import React from 'react';
import Link from '@docusaurus/Link';

/**
 * CrossLink Component
 * Provides consistent linking between different parts of the Isaac Sim curriculum
 */
export default function CrossLink({ to, children, ...props }) {
  return (
    <Link to={to} {...props}>
      {children}
    </Link>
  );
}

/**
 * IsaacSimLink Component
 * Specialized link for Isaac Sim specific content
 */
export function IsaacSimLink({ to, children, variant = 'default', ...props }) {
  const baseClasses = 'isaac-sim-link';
  const variantClass = `isaac-sim-link--${variant}`;

  return (
    <Link
      to={to}
      className={`${baseClasses} ${variantClass}`}
      {...props}
    >
      {children}
    </Link>
  );
}

/**
 * NavigationHelper Component
 * Provides common navigation patterns for Isaac Sim curriculum
 */
export function NavigationHelper({ type, links }) {
  switch (type) {
    case 'module-nav':
      return (
        <div className="module-navigation">
          {links.map((link, index) => (
            <div key={index} className="module-nav-item">
              <Link to={link.url}>{link.title}</Link>
              <span className="nav-description">{link.description}</span>
            </div>
          ))}
        </div>
      );
    default:
      return null;
  }
}