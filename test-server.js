// test-server.js
const express = require('express');
const app = express();

app.use(express.json());

// Simple test endpoint
app.get('/', (req, res) => {
  res.json({ message: 'Context7 MCP Server is running' });
});

// Context7 file operation API endpoint
app.post('/api/v1/file-operation', (req, res) => {
  console.log('Received file operation:', req.body);

  // Mock successful response
  res.json({
    success: true,
    operation: req.body.operation,
    path: req.body.path,
    message: 'File operation completed successfully',
    timestamp: new Date().toISOString()
  });
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'ok', timestamp: new Date().toISOString() });
});

const PORT = process.env.PORT || 3001;
app.listen(PORT, () => {
  console.log(`Test Context7 MCP server running on port ${PORT}`);
  console.log(`Test endpoints:`);
  console.log(`  GET  / - Health check`);
  console.log(`  POST /api/v1/file-operation - File operations`);
});