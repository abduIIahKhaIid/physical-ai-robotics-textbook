// Accessibility utilities for the Physical AI & Robotics textbook

/**
 * Utility to focus trap within a container
 * Useful for modal dialogs and dropdowns
 */
export const focusTrap = (container: HTMLElement, firstFocus?: HTMLElement) => {
  const focusableElements = container.querySelectorAll(
    'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
  ) as NodeListOf<HTMLElement>;
  
  const firstElement = firstFocus || focusableElements[0];
  const lastElement = focusableElements[focusableElements.length - 1];

  const handleKeyDown = (e: KeyboardEvent) => {
    if (e.key !== 'Tab') return;

    if (e.shiftKey && document.activeElement === firstElement) {
      lastElement.focus();
      e.preventDefault();
    } else if (!e.shiftKey && document.activeElement === lastElement) {
      firstElement.focus();
      e.preventDefault();
    }
  };

  container.addEventListener('keydown', handleKeyDown);

  // Focus the first element initially
  firstElement?.focus();

  return () => {
    container.removeEventListener('keydown', handleKeyDown);
  };
};

/**
 * Announces a message to screen readers
 */
export const announceToScreenReader = (message: string) => {
  const announcement = document.createElement('div');
  announcement.setAttribute('aria-live', 'polite');
  announcement.setAttribute('aria-atomic', 'true');
  announcement.className = 'sr-only';
  announcement.textContent = message;
  
  document.body.appendChild(announcement);
  
  // Remove after a delay to ensure it's announced
  setTimeout(() => {
    document.body.removeChild(announcement);
  }, 1000);
};

/**
 * Gets the appropriate contrast color based on background
 */
export const getContrastColor = (hexColor: string): 'black' | 'white' => {
  // Convert hex to RGB
  const r = parseInt(hexColor.substring(1, 3), 16);
  const g = parseInt(hexColor.substring(3, 5), 16);
  const b = parseInt(hexColor.substring(5, 7), 16);
  
  // Calculate luminance
  const luminance = (0.299 * r + 0.587 * g + 0.114 * b) / 255;
  
  return luminance > 0.5 ? 'black' : 'white';
};

/**
 * Utility to handle reduced motion preferences
 */
export const prefersReducedMotion = (): boolean => {
  if (typeof window === 'undefined') return false;
  return window.matchMedia('(prefers-reduced-motion: reduce)').matches;
};

/**
 * Utility to handle high contrast mode
 */
export const prefersHighContrast = (): boolean => {
  if (typeof window === 'undefined') return false;
  return window.matchMedia('(prefers-contrast: high)').matches;
};

/**
 * Generate ARIA attributes for progress indicators
 */
export const getProgressAriaProps = (current: number, total: number) => ({
  'aria-valuenow': current,
  'aria-valuemin': 1,
  'aria-valuemax': total,
  role: 'progressbar',
  'aria-label': `Progress: ${current} of ${total}`
});

/**
 * Utility to generate unique IDs for accessibility
 */
let idCounter = 0;
export const generateId = (prefix: string = 'id'): string => {
  idCounter++;
  return `${prefix}-${idCounter}`;
};