# Vercel Deployment Guide

This guide explains how to deploy the Physical AI & Humanoid Robotics Course book to Vercel.

## Quick Deploy

### Option 1: Deploy via Vercel Dashboard (Recommended)

1. **Sign up/Login to Vercel**
   - Go to https://vercel.com
   - Sign up or login (can use GitHub account)

2. **Import Repository**
   - Click "Add New..." → "Project"
   - Select "Import Git Repository"
   - Authorize Vercel to access your GitHub account
   - Select this repository: `Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest`

3. **Configure Project**
   - Framework Preset: Docusaurus 2 (auto-detected)
   - Build Command: `npm run build` (auto-configured)
   - Output Directory: `build` (auto-configured)
   - Install Command: `npm install` (auto-configured)
   - Click "Deploy"

4. **Wait for Deployment**
   - Vercel will automatically build and deploy
   - Takes ~2-3 minutes
   - You'll get a live URL like: `https://your-project.vercel.app`

### Option 2: Deploy via Vercel CLI

1. **Install Vercel CLI**
   ```bash
   npm install -g vercel
   ```

2. **Login to Vercel**
   ```bash
   vercel login
   ```

3. **Deploy**
   ```bash
   vercel
   ```

   Follow the prompts:
   - Set up and deploy: Y
   - Which scope: Your account
   - Link to existing project: N
   - Project name: (default or custom)
   - In which directory: ./
   - Override settings: N

4. **Production Deployment**
   ```bash
   vercel --prod
   ```

## Configuration

The repository is pre-configured with:

- **vercel.json** - Vercel build configuration
- **.vercelignore** - Files to exclude from deployment
- **docusaurus.config.js** - Auto-detects Vercel environment

### Environment Variables (Optional)

If needed, set these in Vercel Dashboard → Settings → Environment Variables:

- `NODE_VERSION`: 18 (default)
- Any custom environment variables for the chatbot backend

## Custom Domain

To use a custom domain:

1. Go to Vercel Dashboard → Your Project → Settings → Domains
2. Add your custom domain
3. Configure DNS records as instructed by Vercel

## Automatic Deployments

Vercel automatically deploys:
- **Production**: Every push to `master` branch
- **Preview**: Every pull request

## Troubleshooting

### Build Fails

Check the build logs in Vercel Dashboard. Common issues:
- Missing dependencies: Run `npm install` locally to verify
- Node version: Ensure Node 18+ is used
- Build command: Should be `npm run build`

### Site Not Loading

- Check the deployment logs
- Verify `baseUrl` in `docusaurus.config.js` is set to `/`
- Clear browser cache and try again

## Benefits of Vercel over GitHub Pages

✅ Faster deployments (2-3 minutes vs 5-10 minutes)
✅ Automatic SSL certificates
✅ Global CDN for better performance
✅ Preview deployments for pull requests
✅ Easy rollbacks
✅ Better build logs and error messages
✅ No workflow configuration needed

## Vercel URL

After deployment, your site will be available at:
- **Production**: `https://your-project.vercel.app`
- **Custom Domain**: Your configured domain (optional)

## Next Steps

After deploying to Vercel:
1. Test the live site
2. Configure custom domain (optional)
3. Set up environment variables for backend services
4. Enable Vercel Analytics (optional)
