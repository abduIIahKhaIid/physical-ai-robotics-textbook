/**
 * Chat widget configuration constants and helpers.
 */

export const MIN_SELECTION_LENGTH = 10;
export const MAX_MESSAGE_LENGTH = 4000;
export const MAX_SELECTED_TEXT_LENGTH = 10000;
export const SESSION_STORAGE_KEY = 'robotutor_session_id';

let _chatApiUrl: string | null = null;

/**
 * Get the chat API base URL from Docusaurus customFields or fallback.
 * Must be called from a React component context or after hydration.
 */
export function getChatApiUrl(): string {
  if (_chatApiUrl) return _chatApiUrl;

  // Try Docusaurus customFields (set at build time)
  try {
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    const {siteConfig} = require('@docusaurus/useDocusaurusContext').default();
    if (siteConfig?.customFields?.chatApiUrl) {
      _chatApiUrl = siteConfig.customFields.chatApiUrl as string;
      return _chatApiUrl;
    }
  } catch {
    // Not in a React context or hook not available
  }

  // Fallback: window global or default
  if (typeof window !== 'undefined' && (window as any).__CHAT_API_URL__) {
    _chatApiUrl = (window as any).__CHAT_API_URL__;
    return _chatApiUrl!;
  }

  return 'https://your-backend.example.com';
}

/**
 * Set the chat API URL (useful for testing or runtime config).
 */
export function setChatApiUrl(url: string): void {
  _chatApiUrl = url;
}
