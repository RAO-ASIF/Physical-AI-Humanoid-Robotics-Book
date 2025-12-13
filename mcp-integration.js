// mcp-integration.js
// Integration layer for MCP servers with Docusaurus documentation generation

const MCPClient = require('./mcp-client');
const config = require('./mcp-config.json');

class DocusaurusMCPIntegration {
  constructor() {
    this.client = new MCPClient(config);
  }

  // Generate a complete Docusaurus documentation site
  async generateDocusaurusSite(siteConfig) {
    console.log('Starting Docusaurus site generation...');

    try {
      // 1. Create basic directory structure
      await this.createDirectoryStructure();

      // 2. Generate configuration files
      await this.client.generateDocusaurusConfig(siteConfig);

      // 3. Create initial documentation pages
      await this.createInitialDocs();

      // 4. Create sidebar configuration
      await this.createSidebarConfig();

      // 5. Create basic components
      await this.createBasicComponents();

      // 6. Create static assets
      await this.createStaticAssets();

      // 7. Create blog posts
      await this.createBlogPosts();

      // 8. Create GitHub Pages workflow
      await this.createGithubWorkflow();

      console.log('Docusaurus site generation completed successfully!');
      return { success: true, message: 'Docusaurus site generated successfully' };
    } catch (error) {
      console.error('Error during Docusaurus site generation:', error);
      throw error;
    }
  }

  async createDirectoryStructure() {
    console.log('Creating directory structure...');

    // The directory structure will be created implicitly when files are generated
    // Each file creation will ensure its parent directories exist
  }

  async createInitialDocs() {
    console.log('Creating initial documentation pages...');

    const docs = [
      {
        path: 'docs/intro.md',
        content: `---
id: intro
title: Introduction
sidebar_position: 1
---

# Introduction

Welcome to our documentation site powered by Docusaurus. This site was generated using AI assistance and MCP integration.
`
      },
      {
        path: 'docs/installation.md',
        content: `---
id: installation
title: Installation
sidebar_position: 2
---

# Installation

Follow these steps to install and set up the project.
`
      },
      {
        path: 'docs/getting-started.md',
        content: `---
id: getting-started
title: Getting Started
sidebar_position: 3
---

# Getting Started

Learn how to get started with our project.
`
      }
    ];

    for (const doc of docs) {
      await this.client.generateDocusaurusFile(doc.path, doc.content);
    }
  }

  async createSidebarConfig() {
    console.log('Creating sidebar configuration...');

    const sidebarContent = `/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    'installation',
    'getting-started',
    {
      type: 'category',
      label: 'Features',
      items: ['features/configuration', 'features/theming', 'features/deployment'],
    },
    {
      type: 'category',
      label: 'Guides',
      items: ['guides/writing-docs', 'guides/sidebar-organization', 'guides/best-practices'],
    },
    {
      type: 'category',
      label: 'Tutorials',
      items: ['tutorials/basic-setup', 'tutorials/advanced-features'],
    },
  ],
};

module.exports = sidebars;
`;

    await this.client.generateDocusaurusFile('sidebars.js', sidebarContent);
  }

  async createBasicComponents() {
    console.log('Creating basic components...');

    // Create the components directory
    const componentContent = `import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Easy to Use',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Docusaurus was designed from the ground up to be easily installed and
        used to get your website up and running quickly.
      </>
    ),
  },
  {
    title: 'Focus on What Matters',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Docusaurus lets you focus on your docs, and we&apos;ll do the chores. Go
        ahead and move your docs into the <code>docs</code> directory.
      </>
    ),
  },
  {
    title: 'Powered by React',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
`;

    await this.client.generateDocusaurusFile('src/components/HomepageFeatures/index.js', componentContent);

    // Create CSS for the component
    const cssContent = `.features {
  display: flex;
  align-items: center;
  padding: 2rem 0;
  width: 100%;
}

.featureSvg {
  height: 200px;
  width: 200px;
}
`;

    await this.client.generateDocusaurusFile('src/components/HomepageFeatures/HomepageFeatures.module.css', cssContent);
  }

  async createStaticAssets() {
    console.log('Creating static assets...');

    // Create a placeholder logo
    const logoContent = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
  <circle cx="50" cy="50" r="40" fill="#469ae0"/>
  <text x="50" y="55" text-anchor="middle" fill="white" font-size="20">D</text>
</svg>
`;

    await this.client.generateDocusaurusFile('static/img/logo.svg', logoContent);

    // Create a favicon placeholder
    const faviconContent = `iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAFbSURBVHgBfZLPa1NRFMZn3jfJtGka26RNW6S1Cgq6EAQX4lYQ3foPiIFKcOHKX8GFK4kL/4qLFrqRKi4F2oUKQqkLbV20qVAw1EJr06Zp3ve9eC8k1iY1yYWZd86P8zvnO4xSiiSJRCLBzs4O8XicYDBIMBjE7/fT0dFBZ2cnkUiEVCpFJpMhFovx9u1bkskkxWKRQqFAoVCgWCySzWbZ2tqS7XK5jFJqIwBAXV0dLpeL/v5+ent76e3tpbu7m+7ubjweDz6fj0gkQjKZJJPJkM1mWVlZYW5ujnQ6TS6Xo1gsUiqVKBQK5PN5stks6XSaVCpFIpGQbQAYHh7m/v37+Hw+hoaGGB4eZmhoiJ6eHnw+HwMDA4yNjTE5Ocnk5CQTEwqFmJiYYGJigvHxcSYmJpiYmGBiYoJIJEI4HCYcDhMIBPB4PLK9s7PD3t4eAD6fj4GBAfx+P4FAQD7H43EymQzZbJZcLkc2myWbzZLNZtnb2yMSibC9vc3W1hbb29tsbW2xsbHB2toaKysrrK6usrKysgFgb2+P3d1dLMsiEAjQ19dHX18fPp+P3t5eent76enpwev14vV68fl8DA0NMTw8zPDwMCMjI4yMjDA6OsrY2BhjY2OMjY0xOjrK8PAwfr+fvr4+ent78Xg8eDweXC4Xbrcbj8cj32cymY2dnR2KxSKFQoFCobABwLZtisUi5XJ5A0Aul9sAsG2bXC5HKpUikUgQj8eJxWJsbn7FMAyWlpZYXFxkaWmJhYUF5ufnmZubY3Z2lq9fvzIzM8P09DTT09N8/vyZT58+8eHDB96/f8/bt2958+YNr1+/5tWrV7x8+ZIXL17w/PlzJiYmGB8fZ2xsjLGxMcbGxhgbG2N0dJTR0VHGxsaYnJxkcnKS8fFxRkdHGRkZkW0AiqLkP8kPpZf0yiEAAAAASUVORK5CYII=`;

    await this.client.generateDocusaurusFile('static/img/favicon.ico', faviconContent);
  }

  async createBlogPosts() {
    console.log('Creating blog posts...');

    const blogPosts = [
      {
        path: 'blog/2024-01-01-welcome.md',
        content: `---
slug: welcome
title: Welcome
authors: [slorber, yangshun]
tags: [facebook, hello, docusaurus]
---

Congratulations, you've made your first post!

Feel free to poke around and test the various MCP-driven generation capabilities.
`
      },
      {
        path: 'blog/2024-01-02-what-is-docusaurus.md',
        content: `---
slug: what-is-docusaurus
title: What is Docusaurus?
authors: [slorber, yangshun]
tags: [docusaurus, tutorial]
---

Docusaurus is a static-site generator.

It's an excellent tool for writing documentation, blogs, and other content.
`
      }
    ];

    for (const post of blogPosts) {
      await this.client.generateDocusaurusFile(post.path, post.content);
    }
  }

  async createGithubWorkflow() {
    console.log('Creating GitHub Pages workflow...');

    const workflowContent = `name: Deploy to GitHub Pages

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
          github_token: \${{ secrets.GITHUB_TOKEN }}
          # Build output to publish to the \`gh-pages\` branch:
          publish_dir: ./build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to \`gh-pages\` branch:
          # https://github.com/actions/checkout/issues/13#issuecomment-724415212
          # The GH actions bot is used by default if you didn't specify the two fields.
          # You can swap them with your own user credentials.
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
`;

    await this.client.generateDocusaurusFile('.github/workflows/deploy.yml', workflowContent);
  }

  // Method to generate documentation content using MCP
  async generateDocumentationContent(title, content, category = 'guides', position = 1) {
    const filePath = `docs/${category}/${title.toLowerCase().replace(/\s+/g, '-')}.md`;
    const docContent = `---
title: ${title}
sidebar_position: ${position}
---

# ${title}

${content}
`;

    return await this.client.generateDocusaurusFile(filePath, docContent);
  }

  // Method to create a repository using GitHub MCP
  async createRepository(repoName, description) {
    return await this.client.createGithubRepoOperation(repoName, description);
  }

  // Method to commit and push changes using GitHub MCP
  async commitAndPush(files, message) {
    return await this.client.commitAndPushOperation(files, message);
  }
}

module.exports = DocusaurusMCPIntegration;