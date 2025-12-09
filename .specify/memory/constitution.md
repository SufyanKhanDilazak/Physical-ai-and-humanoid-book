<!-- SYNC IMPACT REPORT
Version change: N/A (initial version) → 1.0.0
Added sections: All principles and governance sections for AI-Native Robotics project
Removed sections: None (this is the initial version)
Templates requiring updates: ✅ Updated
Follow-up TODOs: None
-->

# AI-Native Robotics Book + Embedded RAG Chatbot Constitution

## Core Principles

### I. High Accuracy and Verified Sources (NON-NEGOTIABLE)
All content must use verified and primary robotics & AI sources. Information accuracy is paramount, with all claims traceable to credible academic or industry sources. Code examples must be fully tested and validated in simulation environments.

### II. Academic Writing Standards
Content must follow clear academic writing standards appropriate for CS students at Grade 10–12 level. All diagrams, ROS graphs, examples, and code blocks must be clearly explained and accessible. APA citation style is mandatory for all references.

### III. Full Reproducibility
All code, simulations, and deployments must be fully reproducible. Every tutorial and example must include step-by-step instructions with specific commands and expected outputs. Dependencies and environment configurations must be documented and versioned.

### IV. Rigorous Citation and Academic Integrity
All content must follow rigorous citation standards: APA style, minimum 15 sources per major section (with 50% peer-reviewed), and zero tolerance for plagiarism. All claims must be supported by evidence from cited sources.

### V. Docusaurus Book Architecture
The book must be built with Spec-Kit Plus + Claude Code, structured around Physical AI & Humanoid Robotics curriculum. Include interactive diagrams, ROS graphs, code examples, and step-by-step tutorials with clear learning objectives.

### VI. RAG Chatbot Excellence
The embedded RAG chatbot must use only book content (and optionally user-selected text) as its knowledge base. The stack (FastAPI, Neon Postgres, Qdrant) must be properly integrated with chunking, retrieval, grounding, and Agent Tools. The chatbot must be embedded directly inside the Docusaurus UI.

## Additional Constraints

Technology stack: Docusaurus for book deployment to GitHub Pages, OpenAI Agents/ChatKit, FastAPI backend, Neon Postgres, and Qdrant vector DB for the RAG chatbot. All dependencies must be properly documented and maintained. Code examples must be compatible with ROS 2, Isaac Sim, and Gazebo simulation environments.

## Module-Specific Requirements

### Module 1 — The Robotic Nervous System (ROS 2)
Implementation must include Nodes, Topics, Services, DDS basics, rclpy bridging for Python agents, and URDF creation for humanoids. Outcome: Working ROS 2 package + complete humanoid URDF.

### Module 2 — The Digital Twin (Gazebo & Unity)
Implementation must cover physics simulation (contacts, gravity, collisions), Unity for HRI and visual realism, and simulated sensors (LiDAR, Depth, IMU). Outcome: Humanoid robot simulation + sensor data publishing.

### Module 3 — The AI-Robot Brain (NVIDIA Isaac)
Implementation must include Isaac Sim for photorealistic environments + synthetic data, Isaac ROS for VSLAM/navigation, and Nav2 for bipedal path planning. Outcome: ROS–Isaac bridge + working VSLAM + Nav2 demo.

### Module 4 — Vision-Language-Action (VLA)
Implementation must include Whisper for voice → commands, LLM plans → ROS 2 action sequences, and end-to-end perception → planning → manipulation pipeline. Outcome: Humanoid robot completing a natural-language mission.

## Capstone Requirements

The capstone project must integrate all modules: Robot receives a voice command → plans steps → navigates → identifies object → manipulates it in simulation. All components must work seamlessly together in the simulation environment.

## Governance

This constitution supersedes all other practices and must be followed by all team members. Amendments require documentation of the change, approval from project maintainers, and a migration plan if needed. All pull requests and code reviews must verify compliance with these principles. The project must maintain academic integrity and technical excellence throughout all development phases.

Version: 1.0.0 | Ratified: 2025-12-09 | Last Amended: 2025-12-09
