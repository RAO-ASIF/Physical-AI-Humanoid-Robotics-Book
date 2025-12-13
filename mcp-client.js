// mcp-client.js
// Client for handling MCP server communications

class MCPClient {
  constructor(config) {
    this.context7Config = config.context7;
    this.githubMcpConfig = config.githubMcp;
    this.context7Schema = config.schemas.context7;
    this.githubMcpSchema = config.schemas.githubMcp;
  }

  // Context7 MCP server methods
  async createContext7FileOperation(filePath, content) {
    const operation = {
      operation: 'create',
      path: filePath,
      content: content,
      schema_version: this.context7Schema,
      timestamp: new Date().toISOString()
    };

    return this.sendToContext7Server(operation);
  }

  async updateContext7FileOperation(filePath, content) {
    const operation = {
      operation: 'update',
      path: filePath,
      content: content,
      schema_version: this.context7Schema,
      timestamp: new Date().toISOString()
    };

    return this.sendToContext7Server(operation);
  }

  async sendToContext7Server(operation) {
    try {
      const response = await fetch(`${this.context7Config.serverUrl}/api/${this.context7Config.apiVersion}/file-operation`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        },
        body: JSON.stringify(operation)
      });

      if (!response.ok) {
        throw new Error(`Context7 server error: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error communicating with Context7 server:', error);
      throw error;
    }
  }

  // GitHub MCP server methods
  async createGithubRepoOperation(repoName, description, visibility = 'public') {
    const operation = {
      operation: 'create-repo',
      repo_name: repoName,
      description: description,
      visibility: visibility,
      schema_version: this.githubMcpSchema,
      timestamp: new Date().toISOString()
    };

    return this.sendToGithubMcpServer(operation);
  }

  async commitAndPushOperation(files, message, branch = 'main') {
    const operation = {
      operation: 'commit-push',
      files: files,
      message: message,
      branch: branch,
      schema_version: this.githubMcpSchema,
      timestamp: new Date().toISOString()
    };

    return this.sendToGithubMcpServer(operation);
  }

  async sendToGithubMcpServer(operation) {
    try {
      const response = await fetch(`${this.githubMcpConfig.serverUrl}/api/${this.githubMcpConfig.apiVersion}/github-operation`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        },
        body: JSON.stringify(operation)
      });

      if (!response.ok) {
        throw new Error(`GitHub MCP server error: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error communicating with GitHub MCP server:', error);
      throw error;
    }
  }

  // Integration methods for Docusaurus documentation generation
  async generateDocusaurusFile(filePath, content) {
    // Validate file path for Docusaurus structure
    if (!this.isValidDocusaurusPath(filePath)) {
      throw new Error(`Invalid Docusaurus file path: ${filePath}`);
    }

    // Send to Context7 server for file generation
    return await this.createContext7FileOperation(filePath, content);
  }

  async generateDocusaurusConfig(config) {
    const configContent = this.generateDocusaurusConfigContent(config);
    return await this.generateDocusaurusFile('docusaurus.config.ts', configContent);
  }

  isValidDocusaurusPath(filePath) {
    const validPaths = [
      /^docs\/.*\.(md|mdx)$/,
      /^src\/.*\.(js|jsx|ts|tsx|css)$/,
      /^static\/.*$/,
      /^blog\/.*\.(md|mdx)$/,
      /^.*\.(ts|js|json)$/
    ];

    return validPaths.some(pattern => pattern.test(filePath));
  }

  generateDocusaurusConfigContent(config) {
    // Generate TypeScript configuration for Docusaurus
    return `import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: '${config.title || 'My Site'}',
  tagline: '${config.tagline || 'Dinosaurs are cool'}',
  favicon: '${config.favicon || 'img/favicon.ico'}',

  // Set the production url of your site here
  url: '${config.url || 'https://your-project.github.io'}',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages deployment, it's usually '/<project-name>/'
  baseUrl: '${config.baseUrl || '/'}',

  // GitHub pages deployment config.
  organizationName: '${config.organizationName || 'your-username'}', // Usually your GitHub org/user name.
  projectName: '${config.projectName || 'my-docusaurus-docs'}', // Usually your repo name.

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
        title: '${config.navbarTitle || 'My Site'}',
        logo: {
          alt: '${config.logoAlt || 'Logo'}',
          src: '${config.logoSrc || 'img/logo.svg'}',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: '${config.navbarLabel || 'Tutorial'}',
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
        copyright: \`Copyright © \${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.\`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

module.exports = config;`;
  }
}

module.exports = MCPClient;