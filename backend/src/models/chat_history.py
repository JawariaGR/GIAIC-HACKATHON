from datetime import datetime
from typing import Optional, List
from uuid import UUID, uuid4

from pydantic import BaseModel, Field
from enum import Enum

# Define Enums for chat mode and sender role
class ChatMode(str, Enum):
    FULL_BOOK = "full_book"
    SELECTION = "selection"

class SenderRole(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"

# Pydantic models for API and data validation
class ChatMessage(BaseModel):
    message_id: UUID = Field(default_factory=uuid4)
    session_id: UUID
    sender: SenderRole
    text: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    context_used: Optional[List[str]] = None

class ChatSession(BaseModel):
    session_id: UUID = Field(default_factory=uuid4)
    user_id: str # Anonymous user identifier
    start_time: datetime = Field(default_factory=datetime.utcnow)
    last_active_time: datetime = Field(default_factory=datetime.utcnow)
    mode: ChatMode

# --- SQLAlchemy Models (for database interaction) ---
# These are typically defined using SQLAlchemy's declarative base.
# For simplicity and to match the task description of defining 'schema',
# we'll represent them as Pydantic models with an indication of their database intent.
# In a full FastAPI app, you'd have a separate file for SQLAlchemy models
# and Pydantic models would be used for request/response.

# This file focuses on the schema definition as Pydantic models.
# The actual SQLAlchemy ORM models would look something like this (conceptual):
"""
import sqlalchemy
from sqlalchemy import Column, String, DateTime, Enum as SQLEnum, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import declarative_base, relationship

Base = declarative_base()

class ChatSessionDB(Base):
    __tablename__ = "chat_sessions"
    session_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid4)
    user_id = Column(String, nullable=False)
    start_time = Column(DateTime, nullable=False, default=datetime.utcnow)
    last_active_time = Column(DateTime, nullable=False, default=datetime.utcnow)
    mode = Column(SQLEnum(ChatMode), nullable=False)

    messages = relationship("ChatMessageDB", back_populates="session", cascade="all, delete-orphan")

class ChatMessageDB(Base):
    __tablename__ = "chat_messages"
    message_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.session_id"), nullable=False)
    sender = Column(SQLEnum(SenderRole), nullable=False)
    text = Column(String, nullable=False)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow)
    context_used = Column(sqlalchemy.ARRAY(String), nullable=True) # Storing as array of strings

    session = relationship("ChatSessionDB", back_populates="messages")
"""

# For now, this file will contain the Pydantic models for schema definition
# as requested by the task. Actual DB models will be in service layer.
