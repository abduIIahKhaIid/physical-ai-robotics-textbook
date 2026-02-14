// Theme Toggle Script - handles light/dark mode switching
(function() {
  const THEME_KEY = 'theme-mode';
  
  function getStoredTheme() {
    return localStorage.getItem(THEME_KEY);
  }
  
  function setStoredTheme(theme) {
    localStorage.setItem(THEME_KEY, theme);
  }
  
  function getPreferredTheme() {
    const stored = getStoredTheme();
    if (stored) return stored;
    
    return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  }
  
  function applyTheme(theme) {
    const html = document.documentElement;
    if (theme === 'dark') {
      html.setAttribute('data-theme', 'dark');
      html.classList.add('dark');
    } else {
      html.removeAttribute('data-theme');
      html.classList.remove('dark');
    }
    setStoredTheme(theme);
  }
  
  function toggleTheme() {
    const currentTheme = document.documentElement.getAttribute('data-theme') || 'light';
    const newTheme = currentTheme === 'dark' ? 'light' : 'dark';
    applyTheme(newTheme);
  }
  
  // Initialize theme on page load
  function initTheme() {
    const preferredTheme = getPreferredTheme();
    applyTheme(preferredTheme);
    
    // Set up the theme toggle button
    setTimeout(() => {
      const btn = document.getElementById('theme-toggle-btn');
      if (btn) {
        btn.addEventListener('click', toggleTheme);
        // Add hover effect
        btn.addEventListener('mouseenter', (e) => {
          e.currentTarget.style.backgroundColor = 'var(--ifm-color-emphasis-200)';
        });
        btn.addEventListener('mouseleave', (e) => {
          e.currentTarget.style.backgroundColor = 'transparent';
        });
      }
    }, 100);
  }
  
  // Run on DOM ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initTheme);
  } else {
    initTheme();
  }
  
  // Listen for system theme changes
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', (e) => {
    if (!getStoredTheme()) {
      applyTheme(e.matches ? 'dark' : 'light');
    }
  });
})();
