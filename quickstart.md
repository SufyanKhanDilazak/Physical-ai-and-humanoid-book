# Quickstart Guide: Physical AI & Humanoid Robotics Book Development

## Overview
This guide provides the essential steps to get started with developing the Physical AI & Humanoid Robotics book using the Docusaurus platform with RAG integration.

## Prerequisites
- Node.js 18+ installed
- Python 3.8+ installed
- Git installed
- Basic knowledge of ROS 2, Gazebo, and robotics concepts
- GitHub account with repository access

## Setting Up the Development Environment

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-book-source.git
cd physical-ai-book-source
```

### 2. Install Frontend Dependencies
```bash
cd frontend  # Navigate to the Docusaurus directory
npm install
```

### 3. Install Backend Dependencies
```bash
cd backend  # Navigate to the FastAPI directory
pip install -r requirements.txt
```

### 4. Configure Environment Variables
Create a `.env` file in the backend directory:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_db_connection_string
```

## Running the Development Servers

### 1. Start the Frontend (Docusaurus)
```bash
cd frontend
npm start
```
The frontend will be available at http://localhost:3000

### 2. Start the Backend (FastAPI)
```bash
cd backend
uvicorn main:app --reload --port 8000
```
The backend API will be available at http://localhost:8000

## Content Development Workflow

### 1. Adding New Content
1. Create a new markdown file in the appropriate module directory:
   ```
   docs/
   └── module-1-ros2-nervous-system/
       └── new-topic.md
   ```

2. Add the content following the standard Docusaurus format:
   ```markdown
   ---
   title: Your Topic Title
   sidebar_label: Short Label
   description: Brief description of the content
   ---

   # Your Topic Title

   Your content here...

   ## Learning Outcomes
   - Outcome 1
   - Outcome 2

   ## Code Example
   ```python
   # Your code example
   ```
   ```

### 2. Adding Code Examples
When adding code examples, use the following format to ensure reproducibility:
```markdown
import os
# Example Python code for ROS 2

def create_publisher_node():
    """
    Creates a ROS 2 publisher node
    """
    # Your code here
    pass
```

Include information about:
- Required dependencies
- Expected output
- Simulation environment needed
- ROS 2 distribution compatibility

### 3. Updating the RAG System
When you add or modify content, update the RAG system:

1. Chunk the new content:
   ```bash
   python backend/scripts/chunk_content.py --input docs/new-content.md
   ```

2. Index the chunks in Qdrant:
   ```bash
   python backend/scripts/index_chunks.py
   ```

## Testing Your Changes

### 1. Frontend Testing
- Verify all links work correctly
- Check that code examples display properly
- Ensure diagrams and images render correctly
- Test the RAG chatbot integration

### 2. Backend Testing
- Test API endpoints using the built-in Swagger UI at http://localhost:8000/docs
- Verify search functionality
- Test chat responses for accuracy
- Check content retrieval

### 3. Content Accuracy Testing
- Run all code examples in simulation environments
- Verify all citations follow APA format
- Ensure content is appropriate for Grade 10-12 level
- Confirm all claims are traceable to sources

## Deployment Process

### 1. Building the Frontend
```bash
cd frontend
npm run build
```

### 2. Deploying to GitHub Pages
The deployment happens automatically via GitHub Actions when changes are pushed to the main branch. Ensure:

1. All tests pass
2. Content follows academic standards
3. Code examples are reproducible
4. Citations are properly formatted

## Key Tools and Commands

### Docusaurus Commands
- `npm start`: Start development server
- `npm run build`: Build static site
- `npm run serve`: Serve built site locally
- `npm run deploy`: Deploy to GitHub Pages

### Backend Commands
- `uvicorn main:app --reload`: Start development server
- `pytest`: Run backend tests
- `python scripts/test_content.py`: Test content accuracy
- `python scripts/validate_citations.py`: Validate citations

### Content Management
- `python scripts/check_reproducibility.py`: Check if all code examples are reproducible
- `python scripts/validate_academic_standards.py`: Validate academic compliance
- `python scripts/update_rag.py`: Update RAG system with new content

## Academic Standards Compliance

### Citation Requirements
- Every claim must be traceable to a source
- Use APA format for all citations
- Include at least 15 sources per major section (50% peer-reviewed)
- Maintain zero tolerance for plagiarism

### Reproducibility Requirements
- All code examples must run in simulation environments
- Include step-by-step instructions with expected outputs
- Document dependencies and environment configurations
- Version control all examples

## Getting Help

- Check the [main documentation](./docs) for detailed information
- Contact the development team through the project's issue tracker
- Review the [architecture plan](./plan.md) for system design details
- Refer to the [data model](./data-model.md) for content structure information

## Next Steps

1. Set up your development environment
2. Review the module specifications in the `specs/` directory
3. Choose a module to contribute to
4. Follow the content development workflow
5. Test your changes thoroughly
6. Submit a pull request for review