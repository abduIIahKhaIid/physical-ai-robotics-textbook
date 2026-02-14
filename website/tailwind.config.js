/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
  ],
  darkMode: ['class', '[data-theme="dark"]'], // Activate dark: variants when Docusaurus sets data-theme="dark"
  theme: {
    extend: {
      animation: {
        'float': 'float 6s ease-in-out infinite',
        'blob': 'blob 20s ease-in-out infinite',
      },
      keyframes: {
        float: {
          '0%, 100%': { transform: 'translateY(0px)' },
          '50%': { transform: 'translateY(-20px)' },
        },
        blob: {
          '0%, 100%': { transform: 'translate(0, 0) scale(1)' },
          '25%': { transform: 'translate(20px, -20px) scale(1.1)' },
          '50%': { transform: 'translate(-20px, 20px) scale(0.9)' },
          '75%': { transform: 'translate(10px, 10px) scale(1.05)' },
        },
      },
      backdropBlur: {
        xs: '2px',
      },
    },
  },
  corePlugins: {
    preflight: false, // Disable Tailwind's CSS reset to avoid conflicts with Docusaurus/Infima
  },
  plugins: [],
}