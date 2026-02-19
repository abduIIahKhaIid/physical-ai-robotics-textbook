# UI Patterns for Personalization

This document provides React/TypeScript implementation patterns for the personalization frontend.

## Component Architecture

```
src/
├── components/
│   ├── PersonalizationBadge.tsx     # Shows current personalization
│   ├── PersonalizeButton.tsx        # Chapter-level personalize trigger
│   ├── ContentToggle.tsx            # Switch between personalized/default
│   └── ProfileSummary.tsx           # Display profile in settings
├── hooks/
│   ├── usePersonalization.ts        # Main personalization hook
│   └── useUserProfile.ts            # Profile management
└── utils/
    ├── personalizationEngine.ts     # Client-side rule application
    └── profileStorage.ts            # Local/session storage
```

---

## Core Components

### 1. PersonalizationBadge

Shows what personalization is applied.

```tsx
// components/PersonalizationBadge.tsx
import React from 'react';

interface PersonalizationBadgeProps {
  profile: {
    experience_level: string;
    hardware_tier: string;
    language: string;
  };
  onViewOriginal: () => void;
}

export const PersonalizationBadge: React.FC<PersonalizationBadgeProps> = ({
  profile,
  onViewOriginal
}) => {
  return (
    <div className="personalization-badge">
      <div className="badge-content">
        <span className="badge-icon">✨</span>
        <div className="badge-text">
          <strong>Personalized Content</strong>
          <span className="badge-details">
            {profile.experience_level} • {profile.hardware_tier} hardware • {profile.language.toUpperCase()}
          </span>
        </div>
      </div>
      <button 
        className="view-original-btn"
        onClick={onViewOriginal}
        title="View original content"
      >
        View Original
      </button>
    </div>
  );
};
```

**CSS:**

```css
.personalization-badge {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  padding: 1rem 1.5rem;
  border-radius: 8px;
  margin-bottom: 2rem;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.badge-content {
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

.badge-icon {
  font-size: 1.5rem;
}

.badge-text {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.badge-details {
  font-size: 0.875rem;
  opacity: 0.9;
}

.view-original-btn {
  background: rgba(255, 255, 255, 0.2);
  border: 1px solid rgba(255, 255, 255, 0.3);
  color: white;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  cursor: pointer;
  transition: background 0.2s;
}

.view-original-btn:hover {
  background: rgba(255, 255, 255, 0.3);
}
```

---

### 2. PersonalizeButton

Button at the start of each chapter to trigger personalization.

```tsx
// components/PersonalizeButton.tsx
import React, { useState } from 'react';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalize: () => Promise<void>;
  isPersonalized: boolean;
}

export const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({
  chapterId,
  onPersonalize,
  isPersonalized
}) => {
  const [loading, setLoading] = useState(false);

  const handleClick = async () => {
    setLoading(true);
    try {
      await onPersonalize();
    } finally {
      setLoading(false);
    }
  };

  if (isPersonalized) {
    return (
      <div className="personalized-indicator">
        <span className="check-icon">✓</span>
        Content personalized for you
      </div>
    );
  }

  return (
    <button
      className="personalize-button"
      onClick={handleClick}
      disabled={loading}
    >
      {loading ? (
        <>
          <span className="spinner" />
          Personalizing...
        </>
      ) : (
        <>
          <span className="magic-icon">✨</span>
          Personalize This Chapter
        </>
      )}
    </button>
  );
};
```

**CSS:**

```css
.personalize-button {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border: none;
  padding: 0.75rem 1.5rem;
  border-radius: 6px;
  font-size: 1rem;
  font-weight: 600;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin: 1.5rem 0;
  transition: transform 0.2s, box-shadow 0.2s;
}

.personalize-button:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
}

.personalize-button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.magic-icon {
  font-size: 1.25rem;
}

.spinner {
  border: 2px solid rgba(255, 255, 255, 0.3);
  border-top: 2px solid white;
  border-radius: 50%;
  width: 1rem;
  height: 1rem;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.personalized-indicator {
  background: #10b981;
  color: white;
  padding: 0.75rem 1.5rem;
  border-radius: 6px;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin: 1.5rem 0;
}

.check-icon {
  font-size: 1.25rem;
  font-weight: bold;
}
```

---

### 3. ContentToggle

Toggle between personalized and original content.

```tsx
// components/ContentToggle.tsx
import React from 'react';

interface ContentToggleProps {
  showPersonalized: boolean;
  onToggle: (show: boolean) => void;
}

export const ContentToggle: React.FC<ContentToggleProps> = ({
  showPersonalized,
  onToggle
}) => {
  return (
    <div className="content-toggle">
      <span className="toggle-label">Content View:</span>
      <div className="toggle-buttons">
        <button
          className={`toggle-option ${showPersonalized ? 'active' : ''}`}
          onClick={() => onToggle(true)}
        >
          Personalized
        </button>
        <button
          className={`toggle-option ${!showPersonalized ? 'active' : ''}`}
          onClick={() => onToggle(false)}
        >
          Original
        </button>
      </div>
    </div>
  );
};
```

---

## Hooks

### usePersonalization Hook

Main hook for managing personalization state.

```tsx
// hooks/usePersonalization.ts
import { useState, useEffect, useCallback } from 'react';
import { useUserProfile } from './useUserProfile';

interface PersonalizationState {
  isPersonalized: boolean;
  loading: boolean;
  error: string | null;
  originalContent: string;
  personalizedContent: string;
}

export const usePersonalization = (chapterId: string, defaultContent: string) => {
  const { profile } = useUserProfile();
  const [state, setState] = useState<PersonalizationState>({
    isPersonalized: false,
    loading: false,
    error: null,
    originalContent: defaultContent,
    personalizedContent: defaultContent
  });

  const personalize = useCallback(async () => {
    if (!profile) {
      setState(s => ({ ...s, error: 'Profile not loaded' }));
      return;
    }

    setState(s => ({ ...s, loading: true, error: null }));

    try {
      const response = await fetch('/api/personalize', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          chapterId,
          content: state.originalContent,
          profile
        })
      });

      if (!response.ok) throw new Error('Personalization failed');

      const { personalizedContent } = await response.json();

      setState(s => ({
        ...s,
        isPersonalized: true,
        personalizedContent,
        loading: false
      }));

      // Store preference
      localStorage.setItem(`personalized_${chapterId}`, 'true');
    } catch (error) {
      setState(s => ({
        ...s,
        loading: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      }));
    }
  }, [chapterId, profile, state.originalContent]);

  const reset = useCallback(() => {
    setState(s => ({
      ...s,
      isPersonalized: false,
      personalizedContent: s.originalContent
    }));
    localStorage.removeItem(`personalized_${chapterId}`);
  }, [chapterId]);

  // Check if chapter was previously personalized
  useEffect(() => {
    const wasPersonalized = localStorage.getItem(`personalized_${chapterId}`);
    if (wasPersonalized === 'true' && profile) {
      personalize();
    }
  }, [chapterId, profile, personalize]);

  const currentContent = state.isPersonalized 
    ? state.personalizedContent 
    : state.originalContent;

  return {
    ...state,
    currentContent,
    personalize,
    reset
  };
};
```

---

## Integration with Docusaurus

### Wrapping Chapter Component

```tsx
// src/components/PersonalizedChapter.tsx
import React from 'react';
import { usePersonalization } from '../hooks/usePersonalization';
import { PersonalizationBadge } from './PersonalizationBadge';
import { PersonalizeButton } from './PersonalizeButton';
import { useUserProfile } from '../hooks/useUserProfile';

interface PersonalizedChapterProps {
  chapterId: string;
  children: React.ReactNode;
}

export const PersonalizedChapter: React.FC<PersonalizedChapterProps> = ({
  chapterId,
  children
}) => {
  const { profile } = useUserProfile();
  const { 
    isPersonalized, 
    currentContent, 
    personalize, 
    reset,
    loading 
  } = usePersonalization(chapterId, children?.toString() || '');

  if (!profile) {
    return <>{children}</>;
  }

  return (
    <div className="personalized-chapter">
      {!isPersonalized && (
        <PersonalizeButton
          chapterId={chapterId}
          onPersonalize={personalize}
          isPersonalized={false}
        />
      )}

      {isPersonalized && (
        <PersonalizationBadge
          profile={profile}
          onViewOriginal={reset}
        />
      )}

      <div 
        className="chapter-content"
        dangerouslySetInnerHTML={{ __html: currentContent }}
      />
    </div>
  );
};
```

---

## API Endpoint

### Backend Personalization API

```typescript
// pages/api/personalize.ts
import type { NextApiRequest, NextApiResponse } from 'next';
import { applyPersonalizationRules } from '../../utils/personalizationEngine';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse
) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { chapterId, content, profile } = req.body;

    if (!chapterId || !content || !profile) {
      return res.status(400).json({ error: 'Missing required fields' });
    }

    // Apply personalization rules
    const personalizedContent = await applyPersonalizationRules(
      content,
      profile,
      'config/personalization_rules.json'
    );

    // Cache result
    // await cachePersonalizedContent(chapterId, profile, personalizedContent);

    return res.status(200).json({
      personalizedContent,
      appliedAt: new Date().toISOString()
    });
  } catch (error) {
    console.error('Personalization error:', error);
    return res.status(500).json({ 
      error: 'Personalization failed',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}
```

---

## Performance Optimization

### Caching Strategy

```typescript
// utils/cache.ts
import { LRUCache } from 'lru-cache';

const personalizationCache = new LRUCache<string, string>({
  max: 100, // Cache up to 100 personalized chapters
  ttl: 1000 * 60 * 60, // 1 hour TTL
});

export function getCachedContent(
  chapterId: string,
  profile: any
): string | undefined {
  const cacheKey = `${chapterId}_${JSON.stringify(profile)}`;
  return personalizationCache.get(cacheKey);
}

export function setCachedContent(
  chapterId: string,
  profile: any,
  content: string
): void {
  const cacheKey = `${chapterId}_${JSON.stringify(profile)}`;
  personalizationCache.set(cacheKey, content);
}
```

---

## Accessibility

Ensure personalization features are accessible:

```tsx
// Add ARIA labels
<button
  aria-label="Personalize this chapter based on your learning profile"
  aria-pressed={isPersonalized}
>
  Personalize
</button>

// Announce changes to screen readers
<div role="status" aria-live="polite">
  {isPersonalized && "Content has been personalized for you"}
</div>
```

---

## Testing

### Component Tests

```tsx
// __tests__/PersonalizeButton.test.tsx
import { render, screen, fireEvent } from '@testing-library/react';
import { PersonalizeButton } from '../PersonalizeButton';

test('shows personalize button when not personalized', () => {
  render(
    <PersonalizeButton
      chapterId="ch1"
      onPersonalize={async () => {}}
      isPersonalized={false}
    />
  );

  expect(screen.getByText(/Personalize This Chapter/i)).toBeInTheDocument();
});

test('calls onPersonalize when clicked', async () => {
  const mockPersonalize = jest.fn();
  
  render(
    <PersonalizeButton
      chapterId="ch1"
      onPersonalize={mockPersonalize}
      isPersonalized={false}
    />
  );

  fireEvent.click(screen.getByText(/Personalize This Chapter/i));
  
  expect(mockPersonalize).toHaveBeenCalled();
});
```