# Deployment Guide for ChatKit on GitHub Pages

## Prerequisites

- Docusaurus site configured
- FastAPI backend deployed (Vercel, Railway, or other)
- Backend URL accessible via HTTPS

## Step 1: Configure Docusaurus baseUrl

For GitHub Pages deployment at `https://username.github.io/repo-name/`:

**docusaurus.config.js:**
```javascript
module.exports = {
  url: 'https://username.github.io',
  baseUrl: '/repo-name/',
  // ... rest of config
};
```

For custom domain:
```javascript
module.exports = {
  url: 'https://yourdomain.com',
  baseUrl: '/',
  // ... rest of config
};
```

## Step 2: Environment-Aware Backend URL

Create a config file that adapts to environment:

**src/config/chat.js:**
```javascript
export const getChatConfig = () => {
  const isDev = process.env.NODE_ENV === 'development';
  
  return {
    backendUrl: isDev 
      ? 'http://localhost:8000'
      : 'https://your-backend.vercel.app',
    mode: 'normal', // or 'selection_only'
    authType: 'none' // or 'better-auth'
  };
};
```

## Step 3: Widget Integration

**Option A: Global (Site-Wide)**

Create `src/theme/Layout/index.js`:
```javascript
import React from 'react';
import Layout from '@theme-original/Layout';
import ChatKitWidget from '@site/src/components/ChatKitWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatKitWidget />
    </>
  );
}
```

**Option B: Per-Page**

Add to specific MDX files:
```mdx
import ChatKitWidget from '@site/src/components/ChatKitWidget';

# My Chapter

Content here...

<ChatKitWidget mode="selection_only" />
```

## Step 4: CORS Configuration

Backend must allow GitHub Pages origin:

**main.py:**
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://username.github.io",
        "http://localhost:3000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Step 5: Deploy Backend

### Option A: Vercel

1. Install Vercel CLI: `npm i -g vercel`
2. Create `vercel.json`:
```json
{
  "builds": [
    {
      "src": "main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "main.py"
    }
  ]
}
```
3. Deploy: `vercel --prod`

### Option B: Railway

1. Connect GitHub repo to Railway
2. Add environment variables
3. Railway auto-deploys on push

## Step 6: Build and Deploy Frontend

```bash
# Build Docusaurus
npm run build

# Deploy to GitHub Pages
GIT_USER=<your-username> npm run deploy
```

Or use GitHub Actions:

**.github/workflows/deploy.yml:**
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-node@v2
        with:
          node-version: '18'
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

## Step 7: Verify Deployment

1. Visit your GitHub Pages URL
2. Open browser DevTools (F12)
3. Check Console for errors
4. Test chat functionality
5. Verify selected text capture (if enabled)

## Troubleshooting

### Widget not appearing

- Check browser console for errors
- Verify baseUrl matches GitHub Pages URL
- Ensure component is imported correctly

### CORS errors

- Update backend CORS config with correct origin
- Check protocol (http vs https)
- Verify credentials are allowed

### 404 on API calls

- Check backend URL in config
- Ensure backend is deployed and running
- Verify API routes match

### Selection capture not working

- Ensure mode is 'selection_only'
- Check event listeners are attached
- Verify text length is within limits