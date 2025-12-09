# ADR-002: Deployment Architecture for Docusaurus Book with RAG Backend

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-09
- **Feature:** AI/Spec-Driven Docusaurus Book
- **Context:** Need to select a deployment architecture that separates static frontend from backend services while maintaining cost-effectiveness and scalability for an AI-powered robotics curriculum book.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

**Frontend Deployment**: GitHub Pages
- Platform: GitHub Pages
- Strategy: Static site hosting with CDN distribution
- Repository: Separate repository for deployment artifacts

**Backend Deployment**: Separate hosting service
- Platform: VPS/Cloud hosting service
- Strategy: API service for RAG functionality
- Security: CORS configuration for frontend-backend communication

**Integration Pattern**: Static frontend with external backend API
- Architecture: Clear separation of concerns
- Scalability: Independent scaling of frontend and backend
- Maintenance: Different lifecycle management for static vs dynamic content

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Cost-effective hosting solution using GitHub Pages for static content
- Excellent global CDN performance for static assets
- Clear separation of concerns between frontend and backend
- Independent scaling capabilities for frontend and backend
- Backend can be scaled separately based on RAG usage patterns
- Maintains clean separation between source content and deployed artifacts

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Increased complexity due to cross-origin communication (CORS)
- Additional infrastructure management for backend service
- Potential latency between frontend and backend calls
- More complex CI/CD pipeline to manage two deployment targets
- Requires careful security configuration for API endpoints

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

**Alternative Architecture A**: Netlify/Vercel with server-side capabilities
- Rejected because: Additional services to manage, higher cost for simple static hosting

**Alternative Architecture B**: Self-hosted solution for both frontend and backend
- Rejected because: Higher maintenance burden and infrastructure complexity

**Alternative Architecture C**: AWS S3/CloudFront for static hosting
- Rejected because: Higher cost for static content compared to GitHub Pages

**Alternative Architecture D**: Same repository deployment
- Rejected because: Muddies source code with build artifacts, less flexible CI/CD

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: specs/physical-ai-book/spec.md
- Implementation Plan: plan.md
- Related ADRs: ADR-001 (Technology Stack)
- Evaluator Evidence: research.md <!-- link to eval notes/PHR showing graders and outcomes -->