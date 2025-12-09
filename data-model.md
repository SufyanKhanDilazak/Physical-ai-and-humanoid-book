# Data Model: AI/Spec-Driven Docusaurus Book

## Entity Models

### 1. BookContent
**Description**: Represents a section or page of the book content

**Fields**:
- `id` (string, required): Unique identifier for the content
- `title` (string, required): Title of the content section
- `slug` (string, required): URL-friendly identifier
- `module` (string, required): Module identifier (e.g., "module-1-ros2")
- `chapter` (string, required): Chapter identifier within module
- `content` (string, required): Markdown content
- `metadata` (object, optional): Additional content metadata
  - `author` (string)
  - `reviewers` (array of strings)
  - `tags` (array of strings)
  - `learning_outcomes` (array of strings)
- `version` (string, required): Content version
- `created_at` (datetime, required): Creation timestamp
- `updated_at` (datetime, required): Last update timestamp
- `status` (enum, required): Content status (draft, review, published, archived)

**Relationships**:
- One-to-many with `ContentChunk` (via content_id)
- One-to-many with `CodeExample` (via content_id)

**Validation Rules**:
- `title` must be 1-200 characters
- `slug` must follow URL-safe pattern
- `module` must match defined modules (module-1-ros2, module-2-gazebo, etc.)
- `status` must be one of allowed values
- `content` must not exceed 100KB

### 2. ContentChunk
**Description**: Represents a semantic chunk of content for RAG system

**Fields**:
- `id` (string, required): Unique identifier for the chunk
- `content_id` (string, required): Reference to parent BookContent
- `chunk_index` (integer, required): Order of chunk within content
- `text` (string, required): The chunked text content
- `embedding` (array of numbers, required): Vector embedding for semantic search
- `metadata` (object, optional): Chunk-specific metadata
  - `heading` (string): Associated heading
  - `section_type` (string): Type of content (text, code, diagram)
  - `word_count` (integer): Number of words in chunk
- `created_at` (datetime, required): Creation timestamp
- `updated_at` (datetime, required): Last update timestamp

**Relationships**:
- Many-to-one with `BookContent` (via content_id)

**Validation Rules**:
- `text` must be 50-2000 characters
- `chunk_index` must be non-negative
- `embedding` must have consistent dimensions (1536 for OpenAI ada-002)
- `content_id` must reference existing BookContent

### 3. CodeExample
**Description**: Represents executable code examples within book content

**Fields**:
- `id` (string, required): Unique identifier for the code example
- `content_id` (string, required): Reference to parent BookContent
- `title` (string, required): Descriptive title of the example
- `language` (string, required): Programming language
- `code` (string, required): The actual code content
- `description` (string, optional): Explanation of what the code does
- `expected_output` (string, optional): Expected result when executed
- `dependencies` (array of strings, optional): Required packages/libraries
- `simulation_environment` (string, optional): Required simulation environment
- `created_at` (datetime, required): Creation timestamp
- `updated_at` (datetime, required): Last update timestamp

**Relationships**:
- Many-to-one with `BookContent` (via content_id)

**Validation Rules**:
- `language` must be one of supported languages (python, cpp, bash, xml, urdf)
- `code` must not exceed 5000 characters
- `title` must be 1-100 characters

### 4. UserSession
**Description**: Represents a user session for tracking interaction with the RAG chatbot

**Fields**:
- `id` (string, required): Unique identifier for the session
- `user_id` (string, optional): Identifier for registered users
- `session_token` (string, required): Anonymous session identifier
- `created_at` (datetime, required): Session start timestamp
- `updated_at` (datetime, required): Last activity timestamp
- `expires_at` (datetime, required): Session expiration timestamp
- `metadata` (object, optional): Session-specific metadata
  - `user_agent` (string)
  - `ip_address` (string)
  - `location` (string)

**Relationships**:
- One-to-many with `ChatMessage` (via session_id)

**Validation Rules**:
- `session_token` must be unique and secure
- `expires_at` must be in the future
- `user_id` and `session_token` cannot both be null

### 5. ChatMessage
**Description**: Represents a message in a conversation with the RAG chatbot

**Fields**:
- `id` (string, required): Unique identifier for the message
- `session_id` (string, required): Reference to parent UserSession
- `role` (enum, required): Message role (user, assistant, system)
- `content` (string, required): The message content
- `timestamp` (datetime, required): When the message was created
- `metadata` (object, optional): Message-specific metadata
  - `sources` (array of strings): Content IDs used to generate response
  - `confidence_score` (number): Confidence in the response (0-1)
  - `query_type` (string): Type of query (factual, procedural, conceptual)

**Relationships**:
- Many-to-one with `UserSession` (via session_id)

**Validation Rules**:
- `role` must be one of allowed values
- `content` must be 1-10000 characters
- `confidence_score` must be between 0 and 1 if present

### 6. SearchQuery
**Description**: Represents a search query made through the RAG system

**Fields**:
- `id` (string, required): Unique identifier for the query
- `session_id` (string, required): Reference to UserSession
- `query_text` (string, required): The original search query
- `processed_query` (string, optional): Query after processing/normalization
- `results_count` (integer, required): Number of results returned
- `timestamp` (datetime, required): When the query was made
- `metadata` (object, optional): Query-specific metadata
  - `response_time` (number): Time taken to process query (ms)
  - `retrieved_chunks` (array of strings): IDs of chunks retrieved
  - `search_type` (string): Type of search (semantic, keyword, hybrid)

**Relationships**:
- Many-to-one with `UserSession` (via session_id)

**Validation Rules**:
- `query_text` must be 1-500 characters
- `results_count` must be non-negative
- `response_time` must be positive if present

## State Transitions

### BookContent Status Transitions
- `draft` → `review` (when content is ready for review)
- `review` → `draft` (when changes are requested)
- `review` → `published` (when content is approved)
- `published` → `draft` (when content needs updates)
- Any status → `archived` (when content is deprecated)

### Validation Requirements from Constitution
- All content must support APA citation format (metadata field in BookContent)
- Code examples must be reproducible (dependencies and simulation_environment fields in CodeExample)
- Content must be traceable to sources (sources field in ChatMessage)
- Academic integrity maintained through review process (status field in BookContent)