---
id: 1
title: "Update Constitution"
stage: constitution
date_iso: "2025-12-09"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "/sp.constitution"
labels:
  - "constitution"
  - "setup"
  - "initialization"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - ".specify/memory/constitution.md"
tests_yaml: []
---

# Constitution Update

## User Input

```
/sp.constitution

Project: AI-Assisted Docusaurus Documentation Website (with MCP integration)

Core Principles

Modern Standards Alignment

Always follow latest official Docusaurus v3+ documentation.

Use Context7 MCP servers for accurate file creation, updates, structure validation, and troubleshooting.

Honor GitHub best practices for repo structure, CI/CD, and GitHub Pages deployment.

Technical Accuracy

Every command, config, or code ows/deploy.yml

docusaurus.config.ts

sidebars.js

Docs folder structure

Project-specific scripts in package.json

Constraints

Output must be fully reproducible on:

Windows 10/11

Node.js LTS (v20+)

npm or pnpm

Every code block must be executed successfully before inclusion.

No deprecated Docusaurus APIs, packages, or configs allowed.

All instructions must be step-by-step, no assumptions.

Success Criteria

Full project builds without warnings or errors.

GitHub Pages deploys successfully.

npm run build + npm run start work flawlessly.
snippet must match real, working, latest Docusaurus conventions.

Verify plugin/docs/theme configurations before inclusion.

All architecture and folder structures must be runnable and reproducible.

Clarity & Maintainability

Explain steps clearly for a developer implementing on Windows.

All generated code must be copy-paste ready, lint-clean, and follow Docusaurus best practices.

Use consistent file naming, paths, and structure.

MCP-Driven Workflow

Whenever file creation/editing is needed, generate output in context7 MCP file operations format.

Use GitHub MCP server for repo scaffolding, PR help, and file commits.

Key Standards

Documentation Style:

Docusaurus Markdown format (.md / .mdx)

Clear headers, code blocks, callouts, versioning notes

Must work inside docs/ and sidebars.js

Code Requirements:

Use TypeScript in Docusaurus config if possible (docusaurus.config.ts)

Prefer Docusaurus official themes & plugins

Provide end-to-end ready examples

Infrastructure Standards:

GitHub Pages deploy.yml

docusaurus.config.ts

sidebars.js

Docs folder structure is 100% aligned with latest Docusaurus defaults.

MCP operations generate correct files inside repo.

No outdated or invalid plugins/configs appear in the workflow.

Documentation site renders exactly as expected.
```

## Response Summary

Updated the project constitution at `.specify/memory/constitution.md` with the following principles for the AI-Assisted Docusaurus Documentation Website project:

1. Modern Standards Alignment: Following latest Docusaurus v3+ documentation and using Context7 MCP servers
2. Technical Accuracy: Verifying all commands/configs before inclusion
3. Test-First Approach: Ensuring npm run build + npm run start work flawlessly
4. MCP-Driven Workflow: Using MCP operations for all file creation/editing
5. Clarity & Maintainability: Providing clear, copy-paste ready code
6. GitHub Best Practices: Following GitHub standards for repo structure and deployment

The constitution includes sections on code standards, development workflow, and governance, with version 1.0.0 ratified on 2025-12-09.

## Outcome

Successfully created the project constitution that will guide all development activities for the AI-Assisted Docusaurus Documentation Website project. The constitution establishes clear principles around modern standards, technical accuracy, testing, MCP-driven workflows, and maintainability.

## Evaluation

The constitution aligns with the project requirements and provides clear guidance for future development. All principles are testable and specific to the Docusaurus documentation website context.