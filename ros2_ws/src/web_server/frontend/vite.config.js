import { defineConfig } from 'vite';

export default defineConfig({
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
  server: {
    proxy: {
      '/api': 'http://localhost:5000',
      '/stream': 'http://localhost:5000',
      '/gamepad': 'http://localhost:5000',
      '/set_motors': 'http://localhost:5000',
    }
  }
});
