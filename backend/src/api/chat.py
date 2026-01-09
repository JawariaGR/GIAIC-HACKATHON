import logging
import os
from typing import Optional, List
from fastapi import APIRouter, HTTPException, Depends, status
from pydantic import BaseModel
from uuid import UUID
from datetime import datetime



from backend.src.core.rag_retriever import RAGRetriever
from backend.src.models.chat_history import ChatMode
from openai import OpenAI, APIError, AuthenticationError, RateLimitError
from dotenv import load_dotenv

load_dotenv()

router = APIRouter()
logger = logging.getLogger(__name__)

class ChatRequest(BaseModel):
    session_id: Optional[UUID] = None
    user_id: str
    message: str
    mode: ChatMode


class ChatResponse(BaseModel):
    response: str

chat_history_service = None # Explicitly set to None as chat history is removed

# Initialize RAG and OpenAI services
try:
    rag_retriever = RAGRetriever(collection_name="book_rag_collection")
    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    if not openai_client.api_key:
        raise ValueError("OPENAI_API_KEY environment variable not set.")
except ValueError as e:
    logger.error(f"Service initialization error: {e}")
    # For now, let's allow it to proceed but note the error
    rag_retriever = None
    openai_client = None


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

@router.post("/", response_model=ChatResponse, dependencies=[Depends(RateLimiter(times=5, seconds=1))]) # 5 requests per second
async def chat_endpoint(request: ChatRequest):
    if not rag_retriever or not openai_client:
        logger.error("Attempted to use uninitialized services in chat_endpoint.")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Chat service is not fully initialized. Check server logs."
        )



    retrieved_chunks_content: List[str] = []
    context_used_ids: List[str] = []

    try:
        if request.mode == ChatMode.FULL_BOOK:
            retrieved_chunks = await rag_retriever.retrieve_full_book(request.message)
            retrieved_chunks_content = [chunk["content"] for chunk in retrieved_chunks]
            context_used_ids = [chunk["chunk_id"] for chunk in retrieved_chunks]


    except Exception as e:
        logger.exception("Error during RAG retrieval.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error retrieving relevant information."
        )

    ai_response_text = "I cannot find relevant information in the book to answer your question."
    if retrieved_chunks_content:
        ai_response_text = await get_rag_response(request.message, retrieved_chunks_content)



    return ChatResponse(
        response=ai_response_text
    )