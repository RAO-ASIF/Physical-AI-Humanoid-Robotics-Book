---
sidebar_position: 1
title: Introduction to ROS 2 and Physical AI
---

# Introduction to ROS 2 and Physical AI

## Overview

ROS 2 (Robot Operating System 2) serves as the nervous system for humanoid robots, providing the communication backbone that enables different components to work together seamlessly. In the context of Physical AI, ROS 2 is crucial for connecting sensors, actuators, and AI systems in a coordinated manner.

## Key Concepts

### Nodes
Nodes are the fundamental building blocks of ROS 2. Each node performs a specific function and communicates with other nodes through topics, services, and actions.

### Topics
Topics enable unidirectional, asynchronous communication between nodes using a publish/subscribe pattern. This is ideal for sensor data streams and other continuous data flows.

### Services
Services provide bidirectional, synchronous communication using a request/response pattern. This is useful for operations that require a definitive response.

### Actions
Actions enable bidirectional, asynchronous communication for long-running tasks that may provide feedback during execution. This is perfect for navigation and manipulation tasks.

## Why ROS 2 for Physical AI?

ROS 2 provides the infrastructure needed to connect sensors, actuators, and AI systems in a humanoid robot. It handles the complex task of inter-process communication, allowing developers to focus on creating intelligent behaviors rather than managing communication protocols.

## ROS 2 Distributions

For this textbook, we'll be using ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version with extensive documentation and community support. This ensures stability and long-term compatibility for our humanoid robotics projects.

## Getting Started

Before diving into ROS 2 concepts, make sure you have completed the [ROS 2 Setup Guide for Humanoid Robotics](./setup-guide.md) to prepare your development environment.

In the following sections, we'll explore the fundamental concepts of nodes, topics, and services that form the basis of ROS 2 communication, and how they apply specifically to humanoid robotics applications.