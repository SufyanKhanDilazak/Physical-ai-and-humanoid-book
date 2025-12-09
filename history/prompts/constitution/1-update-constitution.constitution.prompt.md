---
id: 1
title: Update Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: Sameer
command: /sp.constitution
labels: [constitution,ai-robotics,rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
---

## Prompt

update my sp.constutition file /sp.constitution

Project: AI-Native Robotics Book + Embedded RAG Chatbot
Output: Docusaurus book deployed to GitHub Pages + RAG chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant.

Core Principles:
- High accuracy using verified/primary robotics & AI sources.
- Clear academic writing (CS students; Grade 10–12).
- Fully reproducible code, simulations, and deployments.
- Rigorous citation: APA style, 15+ sources (50% peer-reviewed), 0% plagiarism.

Book Requirements:
- Built with Spec-Kit Plus + Claude Code.
- Structured around Physical AI & Humanoid Robotics curriculum.
- Include diagrams, ROS graphs, examples, code blocks, and step-by-step tutorials.
- All claims traceable to sources.

RAG Chatbot Requirements:
- Uses only book content (and optionally user-selected text).
- Stack: FastAPI backend, Neon Postgres, Qdrant vector DB.
- Must include chunking, retrieval, grounding, and Agent Tools.
- Embedded directly inside the Docusaurus UI.

Modules:

Module 1 — The Robotic Nervous System (ROS 2)
- Nodes, Topics, Services, DDS basics.
- rclpy bridging for Python agents.
- URDF creation for humanoids.
Outcome: Working ROS 2 package + complete humanoid URDF.

Module 2 — The Digital Twin (Gazebo & Unity)
- Physics simulation (contacts, gravity, collisions).
- Unity for HRI and visual realism.
- Simulated sensors: LiDAR, Depth, IMU.
Outcome: Humanoid robot simulation + sensor data publishing.

Module 3 — The AI-Robot Brain (NVIDIA Isaac)
- Isaac Sim for photorealistic environments + synthetic data.
- Isaac ROS for VSLAM/navigation.
- Nav2 for bipedal path planning.
Outcome: ROS–Isaac bridge + working VSLAM + Nav2 demo.

Module 4 — Vision-Language-Action (VLA)
- Whisper for voice → commands.
- LLM plans → ROS 2 action sequences.
- End-to-end perception → planning → manipulation pipeline.
Outcome: Humanoid robot completing a natural-language mission.

Capstone:
- Robot receives a voice command → plans steps → navigates → identifies object → manipulates it in simulation.

## Response snapshot

Successfully updated the constitution file with the specific requirements for the AI-Native Robotics Book + Embedded RAG Chatbot project. The new constitution includes:

- Core principles focusing on high accuracy, academic writing standards, full reproducibility, and rigorous citation
- Additional constraints for the technology stack (Docusaurus, OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant)
- Module-specific requirements for all four modules (ROS 2, Digital Twin, AI-Robot Brain, VLA)
- Capstone requirements that integrate all modules
- Governance section with project-specific compliance requirements

The constitution now properly reflects the project's academic integrity requirements and technical excellence standards.

## Outcome

- ✅ Impact: Updated project constitution to match AI-Native Robotics requirements
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Continue with project implementation following the new constitution
- 🧠 Reflection: Constitution now properly guides development of the robotics book and RAG chatbot

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing project modules per constitution