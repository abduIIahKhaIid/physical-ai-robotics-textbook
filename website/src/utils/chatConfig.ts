/**
 * Chat widget configuration constants and helpers.
 */

export const MIN_SELECTION_LENGTH = 10;
export const MAX_MESSAGE_LENGTH = 4000;
export const MAX_SELECTED_TEXT_LENGTH = 10000;
export const SESSION_STORAGE_KEY = 'robotutor_session_id';

let _chatApiUrl: string | null = null;

/**
 * Get the chat API base URL.
 * Priority: Codespace auto-detect > Docusaurus customFields > window global > localhost.
 */
export function getChatApiUrl(): string {
  if (_chatApiUrl) return _chatApiUrl;

  // Codespace detection: derive port-8000 URL from current hostname.
  // Must run first because build-time config may contain stale localhost values.
  if (typeof window !== 'undefined' && window.location.hostname.includes('.app.github.dev')) {
    const host = window.location.hostname;
    const backendHost = host.replace(/-\d+\.app\.github\.dev$/, '-8000.app.github.dev');
    _chatApiUrl = `https://${backendHost}`;
    return _chatApiUrl;
  }

  // Try Docusaurus generated site config (works outside React hooks)
  try {
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    const siteConfig = require('@generated/docusaurus.config').default;
    if (siteConfig?.customFields?.chatApiUrl) {
      _chatApiUrl = siteConfig.customFields.chatApiUrl as string;
      return _chatApiUrl;
    }
  } catch {
    // Not in Docusaurus context
  }

  // Fallback: window global
  if (typeof window !== 'undefined' && (window as any).__CHAT_API_URL__) {
    _chatApiUrl = (window as any).__CHAT_API_URL__;
    return _chatApiUrl!;
  }

  return 'http://localhost:8000';
}

/**
 * Set the chat API URL (useful for testing or runtime config).
 */
export function setChatApiUrl(url: string): void {
  _chatApiUrl = url;
}
