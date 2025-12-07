# Feature Tasks: Integrated RAG Chatbot

This document outlines the tasks required to implement the Integrated RAG Chatbot feature. Tasks are organized into phases, with clear dependencies and file paths to facilitate parallel development and incremental delivery.

## Phase 1: Setup

These tasks focus on initializing the project structure and essential configurations.

- [X] T001 Initialize Python backend project structure for FastAPI in `backend/`
- [X] T002 Configure `.gitignore` for backend and website projects
- [X] T003 Setup virtual environment and install core Python dependencies (FastAPI, uvicorn, qdrant-client, openai, psycopg2-binary) in `backend/requirements.txt`
- [X] T004 Install JavaScript dependencies for the website (if needed, e.g., for widget development environment) in `website/package.json`

## Phase 2: Foundational - Book Data Processing

These tasks are prerequisites for the core RAG functionality.

- [X] T005 Implement book content extraction script from markdown files in `backend/scripts/extract_book_content.py`
- [X] T006 Implement text chunking utility in `backend/src/core/text_chunker.py`
- [X] T007 Implement embedding generation utility using OpenAI in `backend/src/core/embedding_generator.py`
- [X] T008 Implement Qdrant client initialization and vector upload utility in `backend/src/services/qdrant_service.py`
- [X] T009 Create a script to execute the full data processing pipeline (extract, chunk, embed, upload) in `backend/scripts/prepare_data.py`

## Phase 3: User Story 1 - Full-Book Q&A [P1]

**Goal**: A reader can ask questions about the entire book and receive accurate answers.
**Independent Test Criteria**: Ask questions about various book topics; verify answers are relevant and correct.

- [X] T010 [US1] Define database schema for chat history in `backend/src/models/chat_history.py`
- [X] T011 [US1] Implement Neon Postgres client and chat history service in `backend/src/services/chat_history_service.py`
- [X] T012 [P] [US1] Implement RAG retrieval logic (full book mode) in `backend/src/core/rag_retriever.py`
- [X] T013 [P] [US1] Implement `POST /chat` endpoint for full-book Q&A in `backend/src/api/chat.py`
- [X] T014 [US1] Develop basic chatbot UI component for `website/src/components/ChatbotWidget/index.tsx`
- [X] T015 [US1] Integrate `POST /chat` API call into the chatbot UI in `website/src/components/ChatbotWidget/index.tsx`
- [X] T016 [US1] Implement displaying answers and citations in the chatbot UI in `website/src/components/ChatbotWidget/index.tsx`

## Phase 4: User Story 2 - Selection-Based Q&A [P2]

**Goal**: A reader can ask questions about a specific selection of text.
**Independent Test Criteria**: Select text, ask questions; verify answers are based *only* on the selection.

- [X] T017 [P] [US2] Extend RAG retrieval logic for selected text mode in `backend/src/core/rag_retriever.py`
- [X] T018 [P] [US2] Implement `POST /chat/selected-text` endpoint for selection-based Q&A in `backend/src/api/chat.py`
- [X] T019 [US2] Implement user text selection listener in `website/src/theme/Layout/index.tsx` (or similar global component)
- [X] T020 [US2] Integrate `POST /chat/selected-text` API call into the chatbot UI in `website/src/components/ChatbotWidget/index.tsx`
- [X] T021 [US2] Add UI toggle/switch for full-book vs. selection mode in `website/src/components/ChatbotWidget/index.tsx`

## Phase 5: User Story 3 - Chat History Review [P3]

**Goal**: A reader can review their previous chat conversations.
**Independent Test Criteria**: Have a conversation, close/reopen chatbot; verify history persists and displays.

- [X] T022 [P] [US3] Implement `GET /history` endpoint to retrieve chat history in `backend/src/api/chat.py`
- [X] T023 [P] [US3] Implement `POST /reset` endpoint to clear chat history in `backend/src/api/chat.py`
- [X] T024 [US3] Implement client-side anonymous user identity management (local storage) in `website/src/components/ChatbotWidget/index.tsx`
- [X] T025 [US3] Integrate `GET /history` into chatbot UI for display in `website/src/components/ChatbotWidget/index.tsx`
- [X] T026 [US3] Implement UI for clearing chat history in `website/src/components/ChatbotWidget/index.tsx`

## Phase 6: Final Integration, Deployment & Polish

These tasks ensure the system is production-ready and fully integrated.

- [X] T027 Implement comprehensive error handling and logging for backend API
- [X] T028 Add rate limiting to backend API endpoints
- [X] T029 Configure Dockerfile for FastAPI backend in `backend/Dockerfile`
- [X] T030 Setup deployment script/configuration for backend to a cloud provider (e.g., render.com, vercel, fly.io)
- [X] T031 Configure environment variables for Neon Postgres, Qdrant, and OpenAI in deployment environment
- [X] T032 Embed the Docusaurus chatbot widget into the book viewing pages in `website/src/theme/DocItem/index.tsx`
- [ ] T033 Conduct end-to-end testing of the RAG chatbot with real book content
- [ ] T034 Optimize embedding and retrieval performance for large book content
- [ ] T035 Implement prompt engineering strategies to improve response quality and handle ambiguity (FR-008)
- [ ] T036 Review and refine UI/UX of the chatbot widget for seamless user experience

## Dependencies

User stories are primarily independent but assume the "Book Data Processing" (Phase 2) is complete.
- Phase 2 must be completed before any User Story Phase (Phase 3, 4, 5).
- Phase 6 (Final Integration) depends on all User Story Phases being completed.

## Parallel Execution Examples

- **User Story 1 (Full-Book Q&A)**:
    - Backend: T010, T011, T012, T013 can be worked on independently.
    - Frontend: T014, T015, T016 can be worked on independently.
- **User Story 2 (Selection-Based Q&A)**:
    - Backend: T017, T018 can be worked on independently.
    - Frontend: T019, T020, T021 can be worked on independently.
- **User Story 3 (Chat History Review)**:
    - Backend: T022, T023 can be worked on independently.
    - Frontend: T024, T025, T026 can be worked on independently.

## Implementation Strategy

An MVP (Minimum Viable Product) would include Phase 1 (Setup), Phase 2 (Book Data Processing), and Phase 3 (User Story 1 - Full-Book Q&A). This provides core functionality and can be incrementally built upon by adding User Story 2 and User Story 3.
