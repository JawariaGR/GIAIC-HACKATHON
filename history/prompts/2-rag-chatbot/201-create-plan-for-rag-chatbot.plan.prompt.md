---
id: 201
title: Create Plan for RAG Chatbot
stage: plan
date: 2025-12-07
surface: agent
model: gemini
feature: 2-rag-chatbot
branch: 2-rag-chatbot
user: 
command: /sp.plan
labels: [planning, rag, chatbot]
links:
  spec: specs/2-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/2-rag-chatbot/plan.md
  - specs/2-rag-chatbot/research.md
  - specs/2-rag-chatbot/data-model.md
  - specs/2-rag-chatbot/contracts/contracts.json
  - specs/2-rag-chatbot/quickstart.md
tests:

---

## Prompt

/sp.plan Goal: Implement a complete Retrieval-Augmented Generation (RAG) chatbot integrated into the SpaceKit book.

Plan:
1. Data Preparation
   - Chunk book text.
   - Generate embeddings.
   - Upload vectors to Qdrant Cloud.

2. Backend Infrastructure
   - Create FastAPI app.
   - Connect to Qdrant for vector search.
   - Connect to Neon Postgres for storing chat history.
   - Build endpoints:
       /chat
       /chat/selected-text
       /history
       /reset

3. Agent Integration
   - Use OpenAI Agents / ChatKit SDK.
   - Configure system prompt + tools for RAG retrieval.
   - Support two retrieval modes:
       a) Full-book semantic search
       b) Strict selected-text extraction

4. Frontend Integration (embedded in book)
   - Add chatbot widget to book UI.
   - Provide text selection UI → send selection to backend.
   - Display responses in chat interface.
   - Handle loading states, errors, and retries.

5. Deployment
   - Deploy FastAPI to serverless hosting.
   - Connect environment variables (OpenAI, Qdrant, Neon).
   - Final QA and testing with real book content.

## Response snapshot

The implementation plan for the RAG chatbot has been successfully generated. The following artifacts were created:
- `specs/2-rag-chatbot/plan.md`: The detailed implementation plan.
- `specs/2-rag-chatbot/research.md`: Research on the technologies to be used.
- `specs/2-rag-chatbot/data-model.md`: The data model for the application.
- `specs/2-rag-chatbot/contracts/contracts.json`: The OpenAPI specification for the API.
- `specs/2-rag-chatbot/quickstart.md`: A quickstart guide for setting up the project.

## Outcome

- ✅ Impact: The plan for the RAG chatbot has been created.
- 🧪 Tests: No tests were run.
- 📁 Files: 5 files were created.
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: The planning phase was successful. The generated artifacts provide a solid foundation for the next phase of development.

## Evaluation notes (flywheel)

- Failure modes observed: The initial attempt to run the setup script failed due to PowerShell execution policy.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
