import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  // GH Pages serves this repo at /garuda-ak-ata-esc/ — see workflow in
  // .github/workflows/deploy-gui.yml.
  base: '/garuda-ak-ata-esc/',
})
