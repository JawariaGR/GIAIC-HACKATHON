# Feature Specification: Integrated RAG Chatbot

**Feature Branch**: `2-rag-chatbot`  
**Created**: 2025-12-07  
**Status**: Draft  
**Input**: Feature: Integrated RAG Chatbot Build and embed a Retrieval-Augmented Generation (RAG) chatbot inside the published book. Requirements: - Use OpenAI Agents / ChatKit SDK. - Backend built with FastAPI. - Neon Serverless Postgres for chat history + analytics. - Qdrant Cloud Free Tier for vector storage and semantic search. - Chatbot must answer: 1. Questions about the entire book. 2. Questions based ONLY on text the user selects. - Pipeline: - Chunk + embed book content. - Store vectors in Qdrant. - Retrieve relevant context (full-book or user-selected). - Generate responses using OpenAI model. - Integrate UI widget inside SpaceKit book viewer. - API must expose endpoints for chat, retrieval mode, and selection-based QA.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Full-Book Q&A (Priority: P1)

A reader wants to understand a concept mentioned in the book without searching manually. They open the chatbot widget, type their question (e.g., "What is a URDF file?"), and receive a concise answer with context drawn from the entire book.

**Why this priority**: This is the core functionality and provides the most immediate value to the user by turning the book into a searchable, interactive knowledge base.

**Independent Test**: Can be tested by asking various questions and verifying the answers are accurate and relevant to the book's content.

**Acceptance Scenarios**:

1. **Given** the chatbot is in "full book" mode, **When** a user asks a question about a topic covered in the book, **Then** the chatbot provides a factually correct answer based on the book's content.
2. **Given** the chatbot is open, **When** a user asks a question about a topic *not* covered in the book, **Then** the chatbot responds that it cannot find an answer within the book.

---

### User Story 2 - Selection-Based Q&A (Priority: P2)

A reader is confused by a specific paragraph or sentence. They highlight the text, right-click, and select an "Ask about this" option. The chatbot then allows them to ask a question *only* about the highlighted text, providing a focused explanation.

**Why this priority**: This enhances comprehension by allowing users to drill down into specific text passages, which is a significant improvement over a general Q&A.

**Independent Test**: Can be tested by selecting different text snippets, asking questions, and confirming the chatbot's answers are strictly based on the selected context.

**Acceptance Scenarios**:

1. **Given** a user has selected a block of text, **When** they ask a question relevant to that text, **Then** the chatbot provides an answer using only the selected text as context.
2. **Given** a user has selected a block of text, **When** they ask a question where the answer is *not* in the selected text (but may be elsewhere in the book), **Then** the chatbot responds that it cannot answer from the given selection.

---

### User Story 3 - Chat History Review (Priority: P3)

A reader wants to review their previous questions and answers. They open the chatbot and can see their conversation history from the current session.

**Why this priority**: Useful for learning and reference, but less critical than the primary Q&A functionalities.

**Independent Test**: Can be tested by having a short conversation and verifying the history is present.

**Acceptance Scenarios**:

1. **Given** a user has had a conversation with the chatbot, **When** they re-open the chatbot widget within the same session, **Then** their previous messages are displayed.

---

### Edge Cases

- What happens when the user highlights a very large block of text? Is there a character limit?
- What happens if the highlighted text contains non-text elements like images or code blocks?
- How does the system handle a rapid succession of questions (rate limiting)?
- What happens if the book content fails to be processed and embedded?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST provide a chatbot interface integrated within the book viewer.
- **FR-002**: The system MUST be capable of processing the entire book's text content for retrieval.
- **FR-003**: The chatbot MUST support a "full book" mode to answer questions using the entire book as context.
- **FR-004**: The chatbot MUST support a "selection" mode to answer questions using only a user-highlighted portion of text as context.
- **FR-005**: The system MUST generate contextually relevant answers to user questions using a large language model.
- **FR-006**: The system MUST persist chat history for each anonymous user for 30 days. The anonymous user identity should be managed via local storage.
- **FR-007**: The user interface MUST provide a visible toggle/switch within the chatbot widget to allow users to manually switch between 'full book' and 'selection' modes.
- **FR-008**: When the chatbot cannot find a relevant answer or the user's question is ambiguous, it MUST ask the user for clarification (e.g., "I'm not sure I understand. Could you rephrase your question?").
- **FR-009**: The system MUST expose an API for submitting chat messages and managing conversation context.

### Key Entities *(include if feature involves data)*

- **BookContentChunk**: A segment of the book's text, used as the basis for retrieval. Contains the raw text and its source location (e.g., chapter, section).
- **ChatSession**: Represents a single, continuous conversation between a user and the chatbot. Contains a list of messages.
- **ChatMessage**: A single message within a ChatSession, including the sender (user or assistant), the message text, and a timestamp.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 90% of user questions about topics explicitly covered in the book receive a factually correct and relevant answer.
- **SC-002**: The average response time, from a user submitting a question to the first word of the answer appearing, is less than 3 seconds.
- **SC-003**: In "selection" mode, 99% of answers are based exclusively on the provided text selection.
- **SC-004**: The system can successfully process and make a 500-page book (approx. 250,000 words) available for querying within 1 hour.
