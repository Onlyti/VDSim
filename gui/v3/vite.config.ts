import path from 'node:path'
import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'

export default defineConfig({
  plugins: [svelte()],
  base: '/v3/',
  resolve: {
    alias: [
      {
        find: '@vdsim/vendor-orbit-controls',
        replacement: path.resolve(
          __dirname,
          '../vendor/three/addons/controls/OrbitControls.js',
        ),
      },
      {
        find: 'three/examples/jsm/',
        replacement: path.resolve(__dirname, '../vendor/three/addons/'),
      },
      {
        find: /^three$/,
        replacement: path.resolve(__dirname, '../vendor/three/three.module.js'),
      },
    ],
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
  server: {
    port: 5173,
    proxy: {
      '/api': { target: 'http://127.0.0.1:8095', changeOrigin: true },
    },
  },
})
