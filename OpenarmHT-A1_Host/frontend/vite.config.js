import { defineConfig } from 'vite';
const target = process.env.OPENARM_BACKEND_URL || 'http://127.0.0.1:5001';
export default defineConfig({server:{host:'127.0.0.1',port:3001,strictPort:true,proxy:{'/api':target,'/model':target}},build:{outDir:'dist'}});
