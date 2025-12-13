// example-usage.js
// Example of how to use the MCP integration for Docusaurus documentation

const DocusaurusMCPIntegration = require('./mcp-integration');

// Example configuration for a Docusaurus site
const siteConfig = {
  title: 'My AI-Assisted Documentation',
  tagline: 'Documentation generated with MCP integration',
  favicon: 'img/favicon.ico',
  url: 'https://your-project.github.io',
  baseUrl: '/my-docusaurus-docs/',
  organizationName: 'your-username',
  projectName: 'my-docusaurus-docs',
  navbarTitle: 'My Docs',
  logoAlt: 'My Logo',
  logoSrc: 'img/logo.svg',
  navbarLabel: 'Docs'
};

// Initialize the MCP integration
const mcpIntegration = new DocusaurusMCPIntegration();

// Example: Generate a complete Docusaurus site
async function generateSite() {
  try {
    console.log('Starting site generation...');
    const result = await mcpIntegration.generateDocusaurusSite(siteConfig);
    console.log('Site generation result:', result);
  } catch (error) {
    console.error('Error generating site:', error);
  }
}

// Example: Generate specific documentation content
async function generateSpecificDocs() {
  try {
    await mcpIntegration.generateDocumentationContent(
      'API Reference',
      'This document provides detailed API reference information.',
      'api',
      1
    );

    await mcpIntegration.generateDocumentationContent(
      'Configuration Guide',
      'Learn how to configure the system properly.',
      'guides',
      4
    );

    console.log('Specific documentation generated successfully!');
  } catch (error) {
    console.error('Error generating specific docs:', error);
  }
}

// Example: Create a GitHub repository
async function createRepo() {
  try {
    const repoResult = await mcpIntegration.createRepository(
      'my-docusaurus-docs',
      'AI-generated documentation site'
    );
    console.log('Repository creation result:', repoResult);
  } catch (error) {
    console.error('Error creating repository:', error);
  }
}

// Example: Commit and push changes
async function commitChanges() {
  try {
    const commitResult = await mcpIntegration.commitAndPush(
      ['docs/intro.md', 'docs/installation.md', 'docusaurus.config.ts'],
      'Initial documentation generation'
    );
    console.log('Commit result:', commitResult);
  } catch (error) {
    console.error('Error committing changes:', error);
  }
}

// Uncomment the function you want to run:
// generateSite();
// generateSpecificDocs();
// createRepo();
// commitChanges();

console.log('MCP Integration example ready. Uncomment the function you want to run.');