/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'relo-dark': '#1a1a1a',
        'relo-panel': '#2d2d2d',
        'relo-blue': '#007bff',
      }
    },
  },
  plugins: [],
}
