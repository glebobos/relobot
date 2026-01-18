import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': {
        target: 'http://localhost:5000',
        changeOrigin: true,
      },
      '/stream': {
        target: 'http://localhost:5000',
        changeOrigin: true,
      },
      '/set_motors': {
        target: 'http://localhost:5000',
        changeOrigin: true,
      },
      '/gamepad': {
        target: 'http://localhost:5000',
        changeOrigin: true,
      },
    }
  }
})
