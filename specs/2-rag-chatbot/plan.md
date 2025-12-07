# Implementation Plan: Integrated RAG Chatbot

**Branch**: `2-rag-chatbot` | **Date**: 2025-12-07 | **Spec**: `/specs/2-rag-chatbot/spec.md`
**Input**: Feature specification from `/specs/2-rag-chatbot/spec.md`

## Summary

This plan outlines the development of an integrated Retrieval-Augmented Generation (RAG) chatbot to be embedded within the published book. The chatbot will leverage OpenAI Agents/ChatKit SDK, a FastAPI backend, Neon Serverless Postgres for chat history and analytics, and Qdrant Cloud Free Tier for vector storage. Its core functionality will include answering questions based on the entire book content and also specifically on user-selected text.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI Agents / ChatKit SDK, Neon Serverless Postgres, Qdrant Cloud Free Tier
**Storage**: Neon Serverless Postgres (chat history, analytics), Qdrant Cloud Free Tier (vector store)
**Testing**: pytest
**Target Platform**: Linux server (backend), Web browser (frontend widget)
**Project Type**: Web application (backend + frontend widget)
**Performance Goals**: Average response time from user query to first word of answer < 3 seconds (SC-002)
**Constraints**: Persist chat history for anonymous users for 30 days (FR-006), chatbot must ask for clarification if unable to answer (FR-008)
**Scale/Scope**: Process a 500-page book (approx. 250,000 words) for querying within 1 hour (SC-004)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **I. Beginner-Friendly & Python-Based**: The backend is Python-based, aligning with beginner-friendly principles and the project's primary language. The chatbot itself will enhance the learning experience.
- [x] **II. Engineering-Focused & Hands-On**: While not a direct lab, implementing a RAG chatbot is a practical engineering task that will provide hands-on experience and valuable functionality.
- [x] **III. Modular & Structured Learning Path**: The chatbot is an integrated feature that enhances existing modules without disrupting the core structure, acting as a cross-cutting concern.
- [x] **IV. End-to-End System Focus**: This feature contributes to an integrated system, potentially interacting with other future AI components (e.g., LLM-based planning).
- [ ] **V. Simulation-First with Digital Twins**: N/A directly for the chatbot's core function, but it processes content that may describe simulated environments. This principle is less relevant here.

## Project Structure

### Documentation (this feature)

```text
specs/2-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   └── core/ # For RAG logic, embeddings, etc.
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

website/ # Existing Docusaurus project
├── src/
│   ├── components/ # New components for chatbot widget
│   └── pages/
└── static/ # Potentially for chatbot assets
```

**Structure Decision**: This project will use a web application structure, extending the existing `website/` (Docusaurus) for the frontend chatbot widget and adding a new `backend/` service for the RAG API.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Principle V: Simulation-First | The RAG chatbot is a software-only feature enhancing the book's learning experience, not directly involved in robotics simulation or digital twin development. Its value lies in information retrieval and interaction. | Not applicable to apply simulation-first here as it is not a robotics component. |
