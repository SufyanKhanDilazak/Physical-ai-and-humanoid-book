# ADR-001: Technology Stack for AI/Spec-Driven Docusaurus Book

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-09
- **Feature:** AI/Spec-Driven Docusaurus Book
- **Context:** Need to select a technology stack that supports building an interactive, AI-powered robotics curriculum book with RAG capabilities, targeting Grade 10-12 students.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

**Documentation Platform**: Docusaurus v3.x
- Framework: Docusaurus (v3.x)
- Features: Excellent documentation features, search capability, plugin architecture for RAG integration

**Backend Stack**: FastAPI + Qdrant + Neon Postgres
- Backend Framework: FastAPI
- Vector Database: Qdrant
- Relational Database: Neon Postgres
- Features: High performance API, semantic search, and data persistence

**AI Integration**: OpenAI Agents/ChatKit with RAG
- Integration: OpenAI Agents/ChatKit for RAG chatbot
- Features: Natural language querying of book content

**Simulation Tools**: ROS 2, Gazebo, Unity, Isaac Sim integration
- Tools: ROS 2, Gazebo, Unity, Isaac Sim
- Features: Integration points for robotics simulation

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Docusaurus provides excellent documentation features with built-in search and plugin architecture
- FastAPI offers excellent performance, automatic API documentation, and good integration with vector databases
- Qdrant is optimized for semantic search with good Python integration
- The stack supports the RAG (Retrieval-Augmented Generation) requirements effectively
- Cost-effective solution using GitHub Pages for static hosting
- Good academic writing standards compliance with Docusaurus

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Multiple technologies to maintain and keep updated
- Separate backend required for RAG functionality adds complexity
- Potential vendor lock-in to specific services
- Learning curve for team members unfamiliar with the stack
- Requires managing multiple services (Docusaurus, FastAPI, Qdrant, Neon Postgres)

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

**Alternative Stack A**: GitBook + LangChain + various backends
- Rejected because: Limited customization options for robotics-specific features

**Alternative Stack B**: Sphinx + Python-focused tools
- Rejected because: Less suitable for mixed tech stack and non-Python content

**Alternative Stack C**: Custom React app
- Rejected because: Higher maintenance burden and development time

**Alternative Stack D**: OpenAI Assistants API
- Rejected because: Less control over data and retrieval process

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: specs/physical-ai-book/spec.md
- Implementation Plan: plan.md
- Related ADRs:
- Evaluator Evidence: research.md <!-- link to eval notes/PHR showing graders and outcomes -->