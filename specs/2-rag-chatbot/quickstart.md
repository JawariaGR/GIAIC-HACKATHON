# Quickstart: Integrated RAG Chatbot

This guide provides instructions to quickly set up and run the Integrated RAG Chatbot.

## Prerequisites

-   Python 3.11+
-   Docker (for backend services like Qdrant, if not using cloud instances during local development)
-   Node.js and npm/yarn (for Docusaurus frontend)
-   OpenAI API Key (for LLM interactions and embeddings)
-   Neon Serverless Postgres connection string
-   Qdrant Cloud API Key and URL (or local Qdrant setup)

## Backend Setup (FastAPI)

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd <repository_name>
    ```

2.  **Create a virtual environment and install dependencies**:
    ```bash
    python -m venv venv
    ./venv/Scripts/activate # On Windows
    source venv/bin/activate # On Linux/macOS
    pip install -r backend/requirements.txt # (Requires backend/requirements.txt to be created)
    ```

3.  **Environment Variables**:
    Create a `.env` file in the `backend/` directory with the following variables:
    ```
    OPENAI_API_KEY="your_openai_api_key"
    NEON_POSTGRES_URL="your_neon_postgres_connection_string"
    QDRANT_URL="your_qdrant_cloud_url"
    QDRANT_API_KEY="your_qdrant_cloud_api_key"
    ```

4.  **Prepare Book Content (Chunking and Embedding)**:
    This step involves processing the book's content, chunking it, generating embeddings, and storing them in Qdrant. A dedicated script will handle this.
    ```bash
    python backend/scripts/prepare_data.py --book_path <path_to_book_content>
    ```

5.  **Run the FastAPI Backend**:
    ```bash
    uvicorn backend.src.main:app --host 0.0.0.0 --port 8000
    ```
    The API will be available at `http://localhost:8000`.

## Frontend Integration (Docusaurus Widget)

1.  **Navigate to the `website` directory**:
    ```bash
    cd website
    ```

2.  **Install dependencies**:
    ```bash
    npm install # or yarn install
    ```

3.  **Configure API Endpoint**:
    The Docusaurus application will need to know the backend API endpoint. This will likely be configured in a `docusaurus.config.js` or similar frontend environment variable. (Details to be determined during implementation).

4.  **Run the Docusaurus Development Server**:
    ```bash
    npm start
    ```
    The book viewer with the integrated chatbot widget will be accessible in your browser (usually `http://localhost:3000`).

## Interacting with the Chatbot

Once both backend and frontend are running:

1.  Open the Docusaurus website in your browser.
2.  Locate and interact with the chatbot widget.
3.  Toggle between "full book" and "selection" modes as described in the feature specification.
4.  Ask questions relevant to the book's content.