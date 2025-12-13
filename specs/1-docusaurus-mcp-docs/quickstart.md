# Quickstart Guide: AI-Assisted Docusaurus Documentation System

## Prerequisites

- Node.js LTS (v20 or higher)
- npm or yarn package manager
- Git for version control
- Access to Context7 MCP server for file generation
- GitHub account for deployment

## Setup Process

### 1. Initialize the Project

First, create a new directory for your documentation project:

```bash
mkdir my-docusaurus-docs
cd my-docusaurus-docs
```

### 2. Install Docusaurus

Use the official Docusaurus CLI to create a new site:

```bash
npm init docusaurus@latest . classic
```

This will create the basic Docusaurus structure with example documentation.

### 3. Configure TypeScript

Update your configuration to use TypeScript by renaming `docusaurus.config.js` to `docusaurus.config.ts` and installing the required type packages:

```bash
mv docusaurus.config.js docusaurus.config.ts
npm install --save-dev @docusaurus/module-type-aliases @docusaurus/types
```

### 4. Project Structure

After setup, your project should have the following structure:

```
my-docusaurus-docs/
├── blog/
│   ├── 2021-08-26-welcome/
│   └── 2019-05-28-first-blog-post.md
├── docs/
│   ├── tutorial-basics/
│   │   ├── congratulations.md
│   │   ├── create-a-blog-post.md
│   │   ├── create-a-document.md
│   │   ├── create-a-page.md
│   │   └── deploy-your-site.md
│   └── intro.md
├── src/
│   ├── css/
│   │   └── custom.css
│   └── pages/
│       ├── index.js
│       └── markdown-page.md
├── static/
│   └── img/
├── docusaurus.config.ts
├── package.json
├── sidebars.js
└── tsconfig.json
```

## MCP Integration

### File Generation via Context7

To generate documentation files using MCP:

1. Prepare your file content following the Context7 schema
2. Send the file operation request to the Context7 server
3. The server will create the file at the specified path with the provided content

Example MCP file operation:
```json
{
  "operation": "create",
  "path": "docs/getting-started.md",
  "content": "# Getting Started\n\nWelcome to our documentation...",
  "schema_version": "context7"
}
```

### GitHub Operations via github-mcp

For repository operations:

1. Initialize repository
2. Commit and push changes
3. Create pull requests

## Configuration

### Docusaurus Configuration (docusaurus.config.ts)

```typescript
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'My Project',
  tagline: 'Dinosaurs are cool',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-project.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages deployment, it's usually '/<project-name>/'
  baseUrl: '/my-docusaurus-docs/',

  // GitHub pages deployment config.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'my-docusaurus-docs', // Usually your repo name.

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
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'My Site',
        logo: {
          alt: 'My Site Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Tutorial',
          },
          {to: '/blog', label: 'Blog', position: 'left'},
          {
            href: 'https://github.com/facebook/docusaurus',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Tutorial',
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
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/facebook/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

module.exports = config;
```

## Documentation Structure

### Creating Documentation Pages

Documentation pages are written in Markdown/MDX and placed in the `docs/` directory. Each page should include frontmatter with metadata:

```markdown
---
title: Page Title
sidebar_position: 1
description: Brief description of the page content
---

# Page Title

Content goes here...
```

### Sidebar Configuration

Update `sidebars.js` to control the navigation structure:

```javascript
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
};

module.exports = sidebars;
```

## Building and Running

### Development

Start the development server:

```bash
npm run start
```

This will start a local server at `http://localhost:3000` with live reloading.

### Production Build

Build the site for production:

```bash
npm run build
```

This creates a `build/` directory with the static site ready for deployment.

### Local Preview

Preview the production build locally:

```bash
npm run serve
```

## GitHub Pages Deployment

### Workflow Configuration

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build

      # Popular action to deploy to GitHub Pages:
      # Docs: https://github.com/peaceiris/actions-gh-pages#%EF%B8%8F-docusaurus
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          # Build output to publish to the `gh-pages` branch:
          publish_dir: ./build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to `gh-pages` branch:
          # https://github.com/actions/checkout/issues/13#issuecomment-724415212
          # The GH actions bot is used by default if you didn't specify the two fields.
          # You can swap them with your own user credentials.
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

### Repository Settings

1. Go to your repository's Settings
2. Navigate to Pages in the left sidebar
3. Select "GitHub Actions" as the source
4. The site will deploy automatically after pushes to the main branch

## Validation Checklist

Before finalizing your documentation site, verify:

- [ ] `npm run build` completes without errors or warnings
- [ ] `npm run start` serves all pages correctly
- [ ] All internal links resolve properly
- [ ] Sidebar navigation works correctly
- [ ] Documentation pages render with proper styling
- [ ] GitHub Pages deployment workflow completes successfully
- [ ] Site is accessible at the configured GitHub Pages URL
- [ ] All configurations follow latest Docusaurus v3+ standards