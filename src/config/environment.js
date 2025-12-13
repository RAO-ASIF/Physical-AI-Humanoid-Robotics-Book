/**
 * Environment Configuration Management for Physical AI & Humanoid Robotics
 *
 * This module provides centralized access to environment variables and configuration
 * settings for the Physical AI & Humanoid Robotics project. It handles validation,
 * defaults, and structured access to configuration values.
 */

class EnvironmentConfig {
  constructor() {
    this.config = this.loadConfig();
    this.validateConfig();
  }

  /**
   * Load configuration from environment variables with defaults
   */
  loadConfig() {
    return {
      // Docusaurus Configuration
      docusaurus: {
        baseUrl: process.env.DOCUSAURUS_BASE_URL || '/physical-ai-humanoid-robotics/',
        title: process.env.DOCUSAURUS_TITLE || 'Physical AI & Humanoid Robotics',
        tagline: process.env.DOCUSAURUS_TAGLINE || 'A Comprehensive Guide to Embodied Intelligence'
      },

      // ROS 2 Configuration
      ros: {
        distro: process.env.ROS_DISTRO || 'humble',
        domainId: parseInt(process.env.ROS_DOMAIN_ID) || 0,
        masterUri: process.env.ROS_MASTER_URI || 'http://localhost:11311'
      },

      // Simulation Environment
      simulation: {
        gazeboModelPath: process.env.GAZEBO_MODEL_PATH || '/usr/share/gazebo/models',
        gazeboWorldPath: process.env.GAZEBO_WORLD_PATH || '/usr/share/gazebo/worlds',
        isaacSimPath: process.env.ISAAC_SIM_PATH || '',
        unityPath: process.env.UNITY_PATH || ''
      },

      // AI/ML Configuration
      ai: {
        openaiApiKey: process.env.OPENAI_API_KEY || '',
        hfApiToken: process.env.HF_API_TOKEN || '',
        cudaDeviceOrder: process.env.CUDA_DEVICE_ORDER || 'PCI_BUS_ID'
      },

      // Development Settings
      development: {
        nodeEnv: process.env.NODE_ENV || 'development',
        debug: process.env.DEBUG === 'true' || process.env.DEBUG === '1',
        logLevel: process.env.LOG_LEVEL || 'info',
        serverPort: parseInt(process.env.SERVER_PORT) || 3000,
        context7McpPort: parseInt(process.env.CONTEXT7_MCP_PORT) || 3001
      }
    };
  }

  /**
   * Validate the loaded configuration
   */
  validateConfig() {
    const errors = [];

    // Validate ROS configuration
    if (!this.config.ros.distro) {
      errors.push('ROS_DISTRO is required');
    }

    // Validate development settings
    if (this.config.development.serverPort <= 0 || this.config.development.serverPort > 65535) {
      errors.push('SERVER_PORT must be between 1 and 65535');
    }

    if (this.config.development.context7McpPort <= 0 || this.config.development.context7McpPort > 65535) {
      errors.push('CONTEXT7_MCP_PORT must be between 1 and 65535');
    }

    // Validate required AI keys for production
    if (this.config.development.nodeEnv === 'production') {
      if (!this.config.ai.openaiApiKey && this.config.ai.requiresOpenAI) {
        errors.push('OPENAI_API_KEY is required for production environment');
      }
    }

    if (errors.length > 0) {
      throw new Error(`Configuration validation errors: ${errors.join(', ')}`);
    }
  }

  /**
   * Get configuration value by path (e.g., 'ros.distro', 'development.nodeEnv')
   */
  get(path) {
    return path.split('.').reduce((obj, key) => obj && obj[key], this.config);
  }

  /**
   * Check if the current environment is production
   */
  isProduction() {
    return this.config.development.nodeEnv === 'production';
  }

  /**
   * Check if the current environment is development
   */
  isDevelopment() {
    return this.config.development.nodeEnv === 'development';
  }

  /**
   * Check if debugging is enabled
   */
  isDebug() {
    return this.config.development.debug;
  }

  /**
   * Get the complete configuration object
   */
  getAll() {
    return { ...this.config };
  }

  /**
   * Check if required AI services are configured
   */
  hasOpenAIConfigured() {
    return !!this.config.ai.openaiApiKey;
  }

  hasHuggingFaceConfigured() {
    return !!this.config.ai.hfApiToken;
  }
}

// Create a singleton instance
const envConfig = new EnvironmentConfig();

// Export both the class and the instance
module.exports = {
  EnvironmentConfig,
  envConfig
};