import logging
import os
from typing import Optional, List
from fastapi import APIRouter, HTTPException, Depends, status
from pydantic import BaseModel
from uuid import UUID
from datetime import datetime

from backend.src.models.chat_history import ChatSession, ChatMessage, ChatMode, SenderRole
from backend.src.services.chat_history_service import ChatHistoryService
from backend.src.core.rag_retriever import RAGRetriever
from openai import OpenAI, APIError, AuthenticationError, RateLimitError
from dotenv import load_dotenv

load_dotenv()

router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services
try:
    chat_history_service = ChatHistoryService()
    rag_retriever = RAGRetriever(qdrant_collection_name="book_rag_collection")
    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    if not openai_client.api_key:
        raise ValueError("OPENAI_API_KEY environment variable not set.")
except ValueError as e:
    logger.error(f"Service initialization error: {e}")
    # In a real application, you might want to stop the app or disable chat functionality
    # For now, let's allow it to proceed but note the error
    chat_history_service = None
    rag_retriever = None
    openai_client = None
    
class ChatRequest(BaseModel):
    session_id: Optional[UUID] = None
    user_id: str
    message: str
    mode: ChatMode
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    session_id: UUID
    message_id: UUID
    response: str
    timestamp: datetime
    context_used: Optional[List[str]] = None

async def get_rag_response(user_message: str, retrieved_context: List[str]) -> str:
    """
    Generates a response from the LLM based on the user message and retrieved context.
    """
    if not openai_client:
        logger.error("OpenAI client not initialized, cannot get RAG response.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Chat service is unavailable due to configuration error."
        )

    context_str = "\n".join(retrieved_context)
    prompt = (
        f"You are a helpful assistant specialized in the provided book content. "
        f"Answer the following question based ONLY on the context provided. "
        f"If you cannot answer the question based solely on the provided context, please state 'I couldn't find a direct answer in the available book content. Would you like me to broaden my search or provide more details?'\n\n"
        f"Context:\n{context_str}\n\n"
        f"Question: {user_message}\n"
        f"Answer:"
    )

    try:
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo", # or gpt-4
            messages=[
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.7
        )
        return response.choices[0].message.content.strip()
    except AuthenticationError as e:
        logger.error(f"OpenAI authentication error: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="OpenAI API authentication failed. Check API key."
        )
    except RateLimitError as e:
        logger.warning(f"OpenAI rate limit exceeded: {e}")
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Too many requests to OpenAI API. Please try again shortly."
        )
    except APIError as e:
        logger.error(f"OpenAI API error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred with the OpenAI API."
        )
    except Exception as e:
        logger.exception("An unexpected error occurred during LLM response generation.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An unexpected error occurred while processing your request."
        )

from fastapi_limiter.depends import RateLimiter

@router.post("/", response_model=ChatResponse)
@router.post("/", response_model=ChatResponse, dependencies=[Depends(RateLimiter(times=5, seconds=1))]) # 5 requests per second
async def chat_endpoint(request: ChatRequest):
    if not chat_history_service or not rag_retriever or not openai_client:
        logger.error("Attempted to use uninitialized services in chat_endpoint.")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Chat service is not fully initialized. Check server logs."
        )

    session: Optional[ChatSession] = None
    try:
        if request.session_id:
            session = await chat_history_service.get_chat_session(request.session_id)

        if not session:
            session = await chat_history_service.create_chat_session(request.user_id, request.mode)
        else:
            await chat_history_service.update_session_last_active_time(session.session_id)
    except Exception as e:
        logger.exception("Error managing chat session.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Could not manage chat session."
        )

    # Store user message
    try:
        user_msg_db = await chat_history_service.add_chat_message(
            session_id=session.session_id,
            sender=SenderRole.USER,
            text=request.message
        )
    except Exception as e:
        logger.exception("Error storing user message.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Could not store user message."
        )

    retrieved_chunks_content: List[str] = []
    context_used_ids: List[str] = []

    try:
        if request.mode == ChatMode.FULL_BOOK:
            retrieved_chunks = await rag_retriever.retrieve_full_book(request.message)
            retrieved_chunks_content = [chunk["content"] for chunk in retrieved_chunks]
            context_used_ids = [chunk["chunk_id"] for chunk in retrieved_chunks]
        elif request.mode == ChatMode.SELECTION:
            if not request.selected_text:
                raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="selected_text is required for SELECTION mode.")
            retrieved_chunks = await rag_retriever.retrieve_selected_text(request.selected_text)
            retrieved_chunks_content = [chunk["content"] for chunk in retrieved_chunks]
            context_used_ids = [chunk["chunk_id"] for chunk in retrieved_chunks]
        else:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid chat mode.")
    except Exception as e:
        logger.exception("Error during RAG retrieval.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error retrieving relevant information."
        )

    ai_response_text = "I cannot find relevant information in the book to answer your question."
    if retrieved_chunks_content:
        ai_response_text = await get_rag_response(request.message, retrieved_chunks_content)

    # Store AI response
    try:
        ai_msg_db = await chat_history_service.add_chat_message(
            session_id=session.session_id,
            sender=SenderRole.ASSISTANT,
            text=ai_response_text,
            context_used=context_used_ids
        )
    except Exception as e:
        logger.exception("Error storing AI response.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Could not store AI response."
        )

    return ChatResponse(
        session_id=session.session_id,
        message_id=ai_msg_db.message_id,
        response=ai_response_text,
        timestamp=ai_msg_db.timestamp,
        context_used=ai_msg_db.context_used
    )

@router.get("/history/{session_id}", response_model=List[ChatMessage])
async def get_history_endpoint(session_id: UUID):
    try:
        history = await chat_history_service.get_chat_history(session_id)
        if not history:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Chat history for session {session_id} not found."
            )
        return history
    except Exception as e:
        logger.exception(f"Error retrieving chat history for session {session_id}.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error retrieving chat history."
        )

@router.post("/reset/{session_id}", status_code=status.HTTP_204_NO_CONTENT)
async def reset_history_endpoint(session_id: UUID):
    try:
        await chat_history_service.clear_chat_history(session_id)
        logger.info(f"Chat history for session {session_id} cleared.")
        return {"message": f"Chat history for session {session_id} cleared."}
    except Exception as e:
        logger.exception(f"Error clearing chat history for session {session_id}.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error clearing chat history."
        )

# Initialize database tables on startup
@router.on_event("startup")
async def startup_event():
    if chat_history_service:
        try:
            await chat_history_service.init_db()
            logger.info("Database tables initialized successfully on startup.")
        except Exception as e:
            logger.error(f"Failed to initialize database tables on startup: {e}")
