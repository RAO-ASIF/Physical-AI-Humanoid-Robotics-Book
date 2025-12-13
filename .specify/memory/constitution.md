<!--
Sync Impact Report:
Version change: 1.0.0 → 2.0.0
Modified principles: Modern Standards Alignment → Spec-Driven Writing, Technical Accuracy (expanded), MCP-Driven Workflow → Tool-Integrated Workflow
Added sections: Clarity for Learners, Version-Controlled Learning, Book Structure Requirements, Automation Standards, Constraints and Technical Requirements
Removed sections: N/A (replaced with new project-specific principles)
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI/Spec-Driven Book Creation Constitution

## Core Principles

### Spec-Driven Writing
All chapters, sections, and components must be generated from clear, modular specs using Spec-Kit Plus. Content generation follows spec-driven development principles where every piece of content is defined by a clear specification before implementation.

### Technical Accuracy
Every claim must be factual, verifiable, and traceable. Use updated AI/robotics/software development sources when needed. Include references when citing research or standards (IEEE/ACM/industry sources). All content must be correct for students of AI, robotics, software engineering, and web development.

### Clarity for Learners
Writing should be accessible to beginner–intermediate tech students, with examples and diagrams where useful. Target clarity level: Flesch-Kincaid grade 8–10. Avoid unnecessary jargon. Use short paragraphs and simple sentence structures. Give summaries at the end of every chapter.

### Consistency
Tone, formatting, terminology, and structure must stay consistent across the entire book. No inconsistent terminology or formatting across chapters. All content follows standardized structure and presentation patterns.

### Version-Controlled Learning
Every update should be reproducible through GitHub commits and Spec-Kit Plus specs. All changes are tracked and versioned appropriately to maintain historical context and enable rollback if needed.

### Tool-Integrated Workflow
All generation must be optimized for Docusaurus Markdown format and Claude Code MCP automation. Use Claude Code for drafting, editing, linting, and consistency checking. All specs must be modular (/sp.chapter, /sp.section, /sp.glossary, etc.).

## Code Standards and Technical Requirements

Documentation Style: Docusaurus Markdown format (.md / .mdx), Clear headers, code blocks, tables, callouts, versioning notes, Must work inside docs/ and sidebars.js. Include diagrams, code blocks, tables, and examples where helpful. Each chapter must have sidebar metadata, frontmatter, and proper file structure.

Content Requirements: Each chapter must include: Overview, Learning objectives, Key concepts, Main content, Examples, Summary, Quiz questions / exercises. All figures or code blocks must follow Docusaurus Markdown formatting. Use MermaidJS diagrams inside Markdown when needed.

Infrastructure Standards: Docusaurus folder structure, sidebars.js configuration, docusaurus.config.js metadata, GitHub Pages deploy.yml, Build-ready for deployment on GitHub Pages, Minimum 10 chapters, Each chapter: 1,200–2,000 words, Book total: 15,000–25,000 words.

## Development Workflow and Automation Standards

Output must be fully reproducible using: Spec-Kit Plus agents, Claude Code, GitHub Pages deployment. All content must be generatable and editable through Spec-Kit Plus agents. Book compiles successfully with no Markdown or config errors. Book deploys correctly to GitHub Pages. All chapters follow the standard structure defined in the principles.

## Governance

This constitution supersedes all other practices and guides all development activities. All PRs/reviews must verify compliance with these principles. Complexity must be justified. Use Spec-Kit Plus and Claude Code as first-class tools for content generation, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual content creation or reliance on internal knowledge.

**Version**: 2.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-10
