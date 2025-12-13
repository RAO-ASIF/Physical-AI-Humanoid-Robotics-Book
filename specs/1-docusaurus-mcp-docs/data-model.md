# Data Model: AI-Assisted Docusaurus Documentation System

## Documentation Entities

### Documentation Page
- **Name**: Individual documentation page (e.g., installation.md, getting-started.md)
- **Fields**:
  - id: string (unique identifier)
  - title: string (page title for display)
  - slug: string (URL-friendly path)
  - content: string (Markdown/MDX content)
  - frontmatter: object (metadata like description, tags, sidebar_position)
  - category: string (parent category/folder)
  - related_pages: array (list of related documentation pages)
- **Relationships**: Belongs to a Category, may link to other Documentation Pages
- **Validation rules**: Must have valid Markdown/MDX syntax, title and slug required
- **State transitions**: Draft → Review → Published

### Documentation Category
- **Name**: Grouping of related documentation pages (e.g., features/, guides/, tutorials/)
- **Fields**:
  - id: string (unique identifier)
  - name: string (category name)
  - description: string (brief description of category)
  - pages: array (list of documentation pages in this category)
  - sidebar_position: number (order in sidebar navigation)
- **Relationships**: Contains multiple Documentation Pages
- **Validation rules**: Name required, must have at least one page or subcategory

### Docusaurus Configuration
- **Name**: Main configuration object for the Docusaurus site
- **Fields**:
  - title: string (site title)
  - tagline: string (site tagline)
  - url: string (deployment URL)
  - baseUrl: string (base URL for the site)
  - organizationName: string (GitHub organization name)
  - projectName: string (GitHub repository name)
  - themeConfig: object (theme-specific configuration)
  - presets: array (Docusaurus presets like docs/blog/blog)
  - plugins: array (additional Docusaurus plugins)
- **Validation rules**: Required fields must be present, URL formats must be valid

### Sidebar Item
- **Name**: Navigation item in the documentation sidebar
- **Fields**:
  - type: string (doc, link, category, html)
  - id: string (reference to documentation page for 'doc' type)
  - label: string (display name in sidebar)
  - href: string (external link for 'link' type)
  - items: array (sub-items for 'category' type)
  - collapsible: boolean (whether category can be collapsed)
  - collapsed: boolean (initial collapsed state)
- **Relationships**: References Documentation Page or contains other Sidebar Items
- **Validation rules**: Required fields based on type, valid references to documentation pages

### Blog Post
- **Name**: Individual blog post
- **Fields**:
  - id: string (unique identifier)
  - title: string (post title)
  - date: string (publication date in YYYY-MM-DD format)
  - authors: array (list of author objects)
  - tags: array (list of tags)
  - description: string (brief description)
  - content: string (Markdown content)
- **Relationships**: None directly, but part of blog collection
- **Validation rules**: Date format must be valid, required fields present

### MCP File Operation
- **Name**: File operation request to MCP server
- **Fields**:
  - operation: string (create, update, delete, read)
  - path: string (file path in repository)
  - content: string (file content for create/update operations)
  - schema_version: string (context7 schema version)
- **Validation rules**: Valid file paths, operation type, content format

## Relationships

### Documentation Hierarchy
- Documentation Category contains multiple Documentation Pages
- Documentation Pages may reference other Documentation Pages as related content
- Sidebar Items reference Documentation Pages or organize other Sidebar Items

### Configuration Relationships
- Docusaurus Configuration contains references to Documentation Categories for sidebar generation
- Docusaurus Configuration defines which Documentation Categories and Pages are included

### MCP Operations
- MCP File Operations are used to generate Documentation Pages, Categories, Configuration, and Sidebar Items
- Generated entities follow the structure defined by MCP operations

## Validation Rules

### Documentation Page Validation
- Must have valid Markdown/MDX syntax
- Title and slug are required fields
- Frontmatter must follow Docusaurus standards
- Content must not contain broken internal links

### Category Validation
- Name must be unique within parent category
- Must have at least one documentation page or subcategory
- Sidebar position must be a valid number

### Configuration Validation
- Required fields (title, tagline, url, etc.) must be present
- URL formats must be valid
- Referenced documentation pages must exist

### Sidebar Validation
- All doc references must point to existing documentation pages
- Category types must have valid items arrays
- No circular references allowed