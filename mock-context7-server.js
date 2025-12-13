// mock-context7-server.js
// A mock implementation of the Context7 MCP server for local development

const express = require('express');
const fs = require('fs').promises;
const path = require('path');
const app = express();

// Middleware to parse JSON
app.use(express.json());

// Store for file operations
const fileStore = new Map();

// Context7 file operation API endpoint
app.post('/api/v1/file-operation', async (req, res) => {
  try {
    console.log('Received file operation:', req.body);

    const { operation, path: filePath, content, schema_version, timestamp } = req.body;

    // Validate required fields
    if (!operation || !filePath) {
      return res.status(400).json({ error: 'Missing required fields: operation and path' });
    }

    // Process the file operation based on the type
    let result;

    if (operation === 'create' || operation === 'update') {
      if (!content) {
        return res.status(400).json({ error: 'Content is required for create/update operations' });
      }

      // Create directory if it doesn't exist
      const dirPath = path.dirname(filePath);
      await fs.mkdir(dirPath, { recursive: true });

      // Write content to file
      await fs.writeFile(filePath, content, 'utf8');

      result = {
        success: true,
        operation: operation,
        path: filePath,
        message: `File ${operation}d successfully`,
        timestamp: timestamp || new Date().toISOString()
      };

      fileStore.set(filePath, { content, timestamp: timestamp || new Date().toISOString() });
    }
    else if (operation === 'read') {
      const fileData = fileStore.get(filePath);
      if (!fileData) {
        return res.status(404).json({ error: 'File not found in mock store' });
      }

      result = {
        success: true,
        operation: operation,
        path: filePath,
        content: fileData.content,
        timestamp: fileData.timestamp
      };
    }
    else if (operation === 'delete') {
      fileStore.delete(filePath);

      // Try to delete the actual file if it exists
      try {
        await fs.unlink(filePath);
      } catch (err) {
        // File might not exist, which is fine
        console.log(`File ${filePath} didn't exist for deletion`);
      }

      result = {
        success: true,
        operation: operation,
        path: filePath,
        message: 'File deleted successfully',
        timestamp: timestamp || new Date().toISOString()
      };
    }
    else {
      return res.status(400).json({ error: `Unsupported operation: ${operation}` });
    }

    console.log('File operation completed:', result);
    res.json(result);
  } catch (error) {
    console.error('Error processing file operation:', error);
    res.status(500).json({
      error: 'Internal server error',
      message: error.message
    });
  }
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'ok', timestamp: new Date().toISOString() });
});

// Start the server
const PORT = process.env.PORT || 3001;
app.listen(PORT, () => {
  console.log(`Mock Context7 MCP server running on port ${PORT}`);
  console.log('API endpoints:');
  console.log(`  POST /api/v1/file-operation - Handle file operations`);
  console.log(`  GET  /health - Health check`);
});

module.exports = app;