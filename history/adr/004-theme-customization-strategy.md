# ADR-004: Theme and Customization Strategy for Docusaurus Book

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-09
- **Feature:** AI/Spec-Driven Docusaurus Book
- **Context:** Need to select an appropriate theme and customization approach that meets academic standards while providing robotics-specific components for the Physical AI & Humanoid Robotics curriculum.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

**Theme Strategy**: Custom theme based on Docusaurus default
- Base: Extend @docusaurus/theme-classic
- Approach: Custom components for robotics-specific content
- Features: Robotics-specific components like URDF viewers, ROS topic diagrams

**Academic Requirements**: Accessibility and learning outcomes tracking
- Compliance: WCAG 2.1 AA compliance
- Features: Screen reader compatibility, keyboard navigation, color contrast
- Tracking: Learning outcome tracking components

**Content Customization**: Specialized components for robotics content
- Components: ROS graphs, simulation screenshots, multi-language code examples
- Branding: Consistency for Physical AI & Humanoid Robotics curriculum
- Styling: Academic styling requirements

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Specialized components for robotics content (ROS graphs, simulation screenshots)
- Academic requirements for accessibility and learning outcomes tracking
- Branding consistency for the Physical AI & Humanoid Robotics curriculum
- Maintains core Docusaurus functionality while adding specialized features
- Can extend default theme while maintaining compatibility
- Component customization allows for robotics-specific elements

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Higher development time compared to using standard theme
- Maintenance burden for custom components
- Potential compatibility issues when updating Docusaurus
- Need for specialized knowledge to maintain custom components
- Additional testing required for custom functionality

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

**Alternative Strategy A**: Standard Docusaurus theme
- Rejected because: Too generic for robotics-specific requirements

**Alternative Strategy B**: Third-party themes
- Rejected because: Lack robotics-specific features needed for curriculum

**Alternative Strategy C**: Complete custom build from scratch
- Rejected because: Too time-intensive and reinventing existing functionality

**Alternative Strategy D**: Minimal customization approach
- Rejected because: Would not meet specialized requirements for robotics content

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