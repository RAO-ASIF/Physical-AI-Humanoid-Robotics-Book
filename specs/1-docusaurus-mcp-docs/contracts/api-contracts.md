# API Contracts: AI-Assisted Docusaurus Documentation System

## MCP File Operations API

### Create Documentation File
- **Endpoint**: POST /mcp/file/create
- **Request**:
  - path: string (target file path, e.g., "docs/introduction.md")
  - content: string (file content in Markdown/MDX format)
  - schema_version: string (context7 schema version)
- **Response**:
  - success: boolean
  - file_path: string (path of created file)
  - message: string (status message)
- **Errors**:
  - 400: Invalid file path or content format
  - 500: Server error during file creation

### Update Documentation File
- **Endpoint**: POST /mcp/file/update
- **Request**:
  - path: string (target file path)
  - content: string (new file content)
  - schema_version: string (context7 schema version)
- **Response**:
  - success: boolean
  - file_path: string (path of updated file)
  - message: string (status message)
- **Errors**:
  - 404: File not found
  - 400: Invalid content format
  - 500: Server error during file update

### Create Docusaurus Configuration
- **Endpoint**: POST /mcp/config/create
- **Request**:
  - config_type: string ("docusaurus.config.ts")
  - settings: object (configuration settings)
  - schema_version: string (context7 schema version)
- **Response**:
  - success: boolean
  - file_path: string (path of created config file)
  - message: string (status message)
- **Errors**:
  - 400: Invalid configuration format
  - 500: Server error during config creation

### Create Sidebar Configuration
- **Endpoint**: POST /mcp/sidebar/create
- **Request**:
  - items: array (sidebar navigation items)
  - schema_version: string (context7 schema version)
- **Response**:
  - success: boolean
  - file_path: string (path of created sidebar file)
  - message: string (status message)
- **Errors**:
  - 400: Invalid sidebar structure
  - 500: Server error during sidebar creation

## GitHub Operations API

### Initialize Repository
- **Endpoint**: POST /mcp/github/init
- **Request**:
  - repo_name: string (repository name)
  - description: string (repository description)
  - visibility: string ("public" or "private")
- **Response**:
  - success: boolean
  - repo_url: string (URL of created repository)
  - message: string (status message)
- **Errors**:
  - 409: Repository already exists
  - 500: Server error during repository creation

### Commit and Push
- **Endpoint**: POST /mcp/github/commit
- **Request**:
  - files: array (list of file paths to include in commit)
  - message: string (commit message)
  - branch: string (target branch)
- **Response**:
  - success: boolean
  - commit_hash: string (hash of created commit)
  - message: string (status message)
- **Errors**:
  - 400: No files to commit
  - 409: Push rejected due to conflicts
  - 500: Server error during commit/push