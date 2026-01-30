# Quickstart Guide: Docusaurus Book Site for Physical AI & Humanoid Robotics

## Prerequisites
- Node.js v18 or higher
- npm package manager
- GitHub account for deployment

## Installation Steps

### 1. Initialize Docusaurus Project
```bash
# Create a new Docusaurus project
npm init docusaurus@latest website classic

# Navigate to project directory
cd website
```

### 2. Install Additional Dependencies
```bash
# Install any additional dependencies if needed
npm install
```

### 3. Configure for GitHub Pages
Edit `docusaurus.config.js` with your specific settings:

```javascript
// docusaurus.config.js
export default {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Educational textbook on advanced robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-org.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/your-repo-name/',

  // GitHub pages deployment config
  organizationName: 'your-org', // Usually your GitHub org/user name
  projectName: 'your-repo-name', // Usually your repo name
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/your-repo-name/tree/main/',
        },
        blog: false, // Disable blog for textbook-only site
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/your-org/your-repo-name',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/facebook/docusaurus',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};
```

### 4. Create GitHub Actions Workflow
Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main
    # Review gh actions docs if you want to further define triggers, paths, etc
    # https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions#on

jobs:
  build:
    name: Build Docusaurus
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build

      - name: Upload Build Artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    name: Deploy to GitHub Pages
    needs: build

    # Grant GITHUB_TOKEN the permissions required to make a Pages deployment
    permissions:
      pages: write # to deploy to Pages
      id-token: write # to verify the deployment originates from an appropriate source

    # Deploy to the github-pages environment
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}

    runs-on: ubuntu-latest
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### 5. Local Development
```bash
# Start local development server
npm start

# Build for production
npm run build

# Serve production build locally for testing
npm run serve
```

## Directory Structure
```
website/
├── blog/                    # Blog posts (optional, disabled for textbook)
├── docs/                   # Documentation files (textbook content)
│   ├── intro.md
│   └── ...
├── src/
│   ├── css/
│   │   └── custom.css
│   └── pages/
│       └── index.js
├── static/                 # Static assets (images, favicon, etc.)
├── docusaurus.config.js    # Site configuration
├── package.json
├── sidebars.js             # Sidebar navigation configuration
└── .github/
    └── workflows/
        └── deploy.yml      # GitHub Actions deployment workflow
```

## Next Steps
1. Add your textbook content to the `docs/` directory
2. Update `sidebars.js` to reflect your content structure
3. Customize the theme and styling in `src/css/custom.css`
4. Add static assets like images and logos to the `static/` directory
5. Commit and push to trigger GitHub Actions deployment