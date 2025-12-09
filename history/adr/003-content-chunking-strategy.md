# ADR-003: Content Chunking Strategy for RAG System

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-09
- **Feature:** AI/Spec-Driven Docusaurus Book
- **Context:** Need to select an effective content chunking strategy for the RAG (Retrieval-Augmented Generation) system that preserves context and meaning while enabling accurate semantic search for robotics curriculum content.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

**Chunking Strategy**: Semantic chunking with content boundary detection and overlap
- Method: Semantic chunking based on content boundaries (sections, chapters) with overlap
- Primary Separators: ['\\n\\n', '\\n', ' ', '']
- Chunk Size: 1000-2000 tokens (balance between context and efficiency)
- Overlap: 200 tokens to maintain context across chunk boundaries
- Special Handling: Custom logic for code blocks and diagrams to keep them intact

**Implementation**: LangChain's RecursiveCharacterTextSplitter with custom separators
- Tool: LangChain's RecursiveCharacterTextSplitter
- Configuration: Custom separators optimized for technical content
- Content Boundary Detection: Respects headings, paragraphs, and code blocks

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Preserves context and meaning within chunks
- Reduces fragmentation of related concepts
- Maintains readability for AI retrieval
- Balances between token efficiency and context preservation
- Outperforms fixed-size chunking for technical content
- Maintains logical groupings through content boundary detection
- Overlap (10-15%) helps maintain context across chunk boundaries
- Works best for robotics content by chunking by conceptual sections

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- More complex implementation than fixed-size chunking
- Requires fine-tuning for optimal separator configuration
- May result in uneven chunk sizes
- Overlap increases storage requirements
- More processing time for chunking operations
- Potential redundancy in retrieved results due to overlap

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

**Alternative Strategy A**: Fixed-size chunking
- Rejected because: Breaks content context and disrupts logical groupings

**Alternative Strategy B**: Document-level chunking
- Rejected because: Too coarse for effective search and retrieval

**Alternative Strategy C**: Sentence-level chunking
- Rejected because: Too fine-grained, increases token usage, loses contextual meaning

**Alternative Strategy D**: Paragraph-level chunking only
- Rejected because: Doesn't handle special content types (code, diagrams) appropriately

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