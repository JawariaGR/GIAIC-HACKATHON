import os
import logging
from datetime import datetime
from typing import List, Optional
from uuid import UUID, uuid4

from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, declarative_base, relationship
from sqlalchemy import Column, String, DateTime, ForeignKey, Text, select, delete, update
from sqlalchemy.dialects.postgresql import UUID as PG_UUID, ARRAY
from sqlalchemy import Enum as SQLEnum

from backend.src.models.chat_history import ChatSession, ChatMessage, ChatMode, SenderRole

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

DATABASE_URL = os.getenv("NEON_POSTGRES_URL")
if not DATABASE_URL:
    logger.error("NEON_POSTGRES_URL environment variable is not set.")
    # In a real app, you might want to raise an exception and prevent startup
    # For now, we'll let it pass and expect errors if DB operations are attempted
    DATABASE_URL = "postgresql+asyncpg://user:password@host:port/dbname" # Dummy URL to avoid immediate crash


Base = declarative_base()

# SQLAlchemy ORM models for database interaction
class ChatSessionDB(Base):
    __tablename__ = "chat_sessions"
    session_id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid4)
    user_id = Column(String, nullable=False)
    start_time = Column(DateTime, nullable=False, default=datetime.utcnow)
    last_active_time = Column(DateTime, nullable=False, default=datetime.utcnow)
    mode = Column(SQLEnum(ChatMode), nullable=False)

    messages = relationship("ChatMessageDB", back_populates="session", cascade="all, delete-orphan", lazy="noload")

    def to_pydantic(self) -> ChatSession:
        return ChatSession(
            session_id=self.session_id,
            user_id=self.user_id,
            start_time=self.start_time,
            last_active_time=self.last_active_time,
            mode=self.mode
        )

class ChatMessageDB(Base):
    __tablename__ = "chat_messages"
    message_id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("chat_sessions.session_id"), nullable=False)
    sender = Column(SQLEnum(SenderRole), nullable=False)
    text = Column(Text, nullable=False) # Use Text for potentially long messages
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow)
    context_used = Column(ARRAY(String), nullable=True)

    session = relationship("ChatSessionDB", back_populates="messages", lazy="noload")

    def to_pydantic(self) -> ChatMessage:
        return ChatMessage(
            message_id=self.message_id,
            session_id=self.session_id,
            sender=self.sender,
            text=self.text,
            timestamp=self.timestamp,
            context_used=self.context_used
        )

class ChatHistoryService:
    def __init__(self):
        if not DATABASE_URL or "user:password@host:port/dbname" in DATABASE_URL:
            logger.warning("Database URL is not properly configured. Chat history functionality will be limited.")
            self.engine = None
            self.AsyncSessionLocal = None
            return

        self.engine = create_async_engine(DATABASE_URL, echo=False) # echo=True for SQL logging
        self.AsyncSessionLocal = sessionmaker(
            self.engine, expire_on_commit=False, class_=AsyncSession
        )

    async def init_db(self):
        """Creates tables if they don't exist."""
        if not self.engine:
            logger.warning("Database engine not initialized. Skipping DB initialization.")
            return
        async with self.engine.begin() as conn:
            # Drop and create all tables - useful for development, be careful in production
            # await conn.run_sync(Base.metadata.drop_all)
            await conn.run_sync(Base.metadata.create_all)
        logger.info("Database tables initialized.")

    async def create_chat_session(self, user_id: str, mode: ChatMode) -> ChatSession:
        if not self.AsyncSessionLocal: raise ConnectionError("Database service not available.")
        async with self.AsyncSessionLocal() as session:
            try:
                new_session_db = ChatSessionDB(user_id=user_id, mode=mode)
                session.add(new_session_db)
                await session.commit()
                await session.refresh(new_session_db)
                logger.info(f"Created new chat session: {new_session_db.session_id} for user {user_id}")
                return new_session_db.to_pydantic()
            except Exception as e:
                logger.exception(f"Error creating chat session for user {user_id}.")
                await session.rollback()
                raise

    async def get_chat_session(self, session_id: UUID) -> Optional[ChatSession]:
        if not self.AsyncSessionLocal: raise ConnectionError("Database service not available.")
        async with self.AsyncSessionLocal() as session:
            try:
                result = await session.execute(
                    select(ChatSessionDB).filter(ChatSessionDB.session_id == session_id)
                )
                session_db = result.scalars().first()
                return session_db.to_pydantic() if session_db else None
            except Exception as e:
                logger.exception(f"Error retrieving chat session {session_id}.")
                raise

    async def add_chat_message(self, session_id: UUID, sender: SenderRole, text: str, context_used: Optional[List[str]] = None) -> ChatMessage:
        if not self.AsyncSessionLocal: raise ConnectionError("Database service not available.")
        async with self.AsyncSessionLocal() as session:
            try:
                new_message_db = ChatMessageDB(
                    session_id=session_id,
                    sender=sender,
                    text=text,
                    timestamp=datetime.utcnow(),
                    context_used=context_used
                )
                session.add(new_message_db)
                
                # Update last_active_time for the session
                await session.execute(
                    update(ChatSessionDB)
                    .where(ChatSessionDB.session_id == session_id)
                    .values(last_active_time=datetime.utcnow())
                )
                
                await session.commit()
                await session.refresh(new_message_db)
                logger.info(f"Added message to session {session_id} from {sender}.")
                return new_message_db.to_pydantic()
            except Exception as e:
                logger.exception(f"Error adding message to session {session_id}.")
                await session.rollback()
                raise

    async def get_chat_history(self, session_id: UUID) -> List[ChatMessage]:
        if not self.AsyncSessionLocal: raise ConnectionError("Database service not available.")
        async with self.AsyncSessionLocal() as session:
            try:
                result = await session.execute(
                    select(ChatMessageDB)
                    .filter(ChatMessageDB.session_id == session_id)
                    .order_by(ChatMessageDB.timestamp)
                )
                messages_db = result.scalars().all()
                return [msg.to_pydantic() for msg in messages_db]
            except Exception as e:
                logger.exception(f"Error retrieving chat history for session {session_id}.")
                raise

    async def clear_chat_history(self, session_id: UUID):
        if not self.AsyncSessionLocal: raise ConnectionError("Database service not available.")
        async with self.AsyncSessionLocal() as session:
            try:
                # Delete all messages associated with the session
                await session.execute(
                    delete(ChatMessageDB).where(ChatMessageDB.session_id == session_id)
                )
                await session.commit()
                logger.info(f"Cleared chat history for session {session_id}.")
            except Exception as e:
                logger.exception(f"Error clearing chat history for session {session_id}.")
                await session.rollback()
                raise

    async def update_session_last_active_time(self, session_id: UUID):
        if not self.AsyncSessionLocal: raise ConnectionError("Database service not available.")
        async with self.AsyncSessionLocal() as session:
            try:
                await session.execute(
                    update(ChatSessionDB)
                    .where(ChatSessionDB.session_id == session_id)
                    .values(last_active_time=datetime.utcnow())
                )
                await session.commit()
                logger.debug(f"Updated last_active_time for session {session_id}.")
            except Exception as e:
                logger.exception(f"Error updating last_active_time for session {session_id}.")
                await session.rollback()
                raise

if __name__ == "__main__":
    async def test_service():
        # Temporarily set a dummy URL for testing if not set
        if not os.getenv("NEON_POSTGRES_URL"):
            os.environ["NEON_POSTGRES_URL"] = "postgresql+asyncpg://user:password@localhost:5432/testdb"
            print("Using dummy NEON_POSTGRES_URL for testing. Please configure your .env file.")

        service = ChatHistoryService()
        if not service.engine:
            print("Service not fully initialized due to missing DB_URL. Cannot run tests.")
            return

        try:
            await service.init_db()

            user_id = "test_user_123"
            mode = ChatMode.FULL_BOOK
            
            # Create a session
            session = await service.create_chat_session(user_id, mode)
            print(f"Created session: {session.session_id}")

            # Add messages
            msg1 = await service.add_chat_message(session.session_id, SenderRole.USER, "Hello chatbot!")
            print(f"Added message: {msg1.text}")
            msg2 = await service.add_chat_message(session.session_id, SenderRole.ASSISTANT, "Hi, how can I help?", ["intro-chunk"])
            print(f"Added message: {msg2.text}")

            # Get history
            history = await service.get_chat_history(session.session_id)
            print("\nChat History:")
            for msg in history:
                print(f"- {msg.sender}: {msg.text}")
            
            # Clear history
            await service.clear_chat_history(session.session_id)
            history_after_clear = await service.get_chat_history(session.session_id)
            print(f"\nHistory after clear: {len(history_after_clear)} messages")
            
        except Exception as e:
            print(f"Test service failed: {e}")
            
    import asyncio
    asyncio.run(test_service())
