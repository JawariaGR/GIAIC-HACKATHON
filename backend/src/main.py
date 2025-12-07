import logging
import sys
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from redis.asyncio import Redis # Changed from redis to redis.asyncio
from fastapi_limiter import FastAPILimiter
from dotenv import load_dotenv

from backend.src.api import chat as chat_router

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6379") # Default to local Redis

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Context manager for managing the lifespan of the FastAPI application.
    Initializes FastAPI-Limiter with Redis on startup and closes the Redis connection on shutdown.
    """
    try:
        redis_instance = Redis.from_url(REDIS_URL, encoding="utf8", decode_responses=True)
        await FastAPILimiter.init(redis_instance)
        logger.info("FastAPI-Limiter initialized with Redis.")
    except Exception as e:
        logger.error(f"Failed to initialize FastAPI-Limiter with Redis: {e}")
        # Decide how to handle this: raise, continue without rate limiting, etc.
        # For now, let's proceed but log the error.
        await FastAPILimiter.init(None) # Initialize with None to prevent further errors

    yield # Application runs

    if FastAPILimiter.redis:
        await FastAPILimiter.redis.close()
        logger.info("FastAPI-Limiter Redis connection closed.")

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Integrated RAG Chatbot.",
    version="1.0.0",
    lifespan=lifespan # Assign the lifespan context manager
)

# Configure CORS
origins = [
    "http://localhost",
    "http://localhost:3000", # Assuming Docusaurus runs on port 3000
    # Add other frontend origins as needed for deployment
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(chat_router.router, prefix="/chat")

@app.get("/")
async def read_root():
    logger.info("Root endpoint accessed.")
    return {"message": "Welcome to the RAG Chatbot API!"}

if __name__ == "__main__":
    logger.info("Starting FastAPI application with Uvicorn.")
    uvicorn.run(app, host="0.0.0.0", port=8000)
