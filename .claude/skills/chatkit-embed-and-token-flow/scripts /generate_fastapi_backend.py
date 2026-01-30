#!/usr/bin/env python3
"""
FastAPI backend template for ChatKit integration.
Supports token-based auth and session management.
"""

import argparse
from pathlib import Path


def generate_fastapi_backend(auth_type: str = "none", db_type: str = "postgres") -> str:
    """Generate FastAPI backend code with proper token/session handling."""
    
    auth_imports = ""
    auth_middleware = ""
    token_endpoint = ""
    
    if auth_type == "better-auth":
        auth_imports = """
from better_auth import BetterAuth
import jwt
from datetime import datetime, timedelta
"""
        auth_middleware = """
auth = BetterAuth()

def create_token(user_id: str) -> str:
    expiration = datetime.utcnow() + timedelta(hours=1)
    payload = {
        'user_id': user_id,
        'exp': expiration
    }
    return jwt.encode(payload, SECRET_KEY, algorithm='HS256')

def verify_token(token: str):
    try:
        return jwt.decode(token, SECRET_KEY, algorithms=['HS256'])
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")
"""
        token_endpoint = """
@app.get("/auth/token")
async def get_token(session: dict = Depends(auth.get_session)):
    # Issue short-lived token for chat session
    token = create_token(session['user_id'])
    return {"token": token}
"""
    
    db_imports = ""
    db_setup = ""
    
    if db_type == "postgres":
        db_imports = """
from sqlalchemy import create_engine, Column, String, Text, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import os
"""
        db_setup = """
# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:pass@localhost/chatkit")
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(bind=engine)
Base = declarative_base()

class ChatMessage(Base):
    __tablename__ = "chat_messages"
    
    id = Column(String, primary_key=True)
    session_id = Column(String, index=True)
    message = Column(Text)
    response = Column(Text)
    selected_text = Column(Text, nullable=True)
    timestamp = Column(DateTime, default=datetime.utcnow)

Base.metadata.create_all(engine)
"""
    
    return f'''from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import uuid
from datetime import datetime
{auth_imports}
{db_imports}

app = FastAPI(title="ChatKit Backend")

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

{"SECRET_KEY = 'your-secret-key-change-in-production'" if auth_type == "better-auth" else ""}
{auth_middleware}
{db_setup}

# Request models
class ChatRequest(BaseModel):
    message: str
    session_id: str
    mode: str = "normal"
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    session_id: str
    message_id: str

# Session storage (use Redis or DB in production)
sessions = {{}}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    {"token: dict = Depends(verify_token)" if auth_type == "better-auth" else ""}
):
    # Validate request
    if len(request.message.strip()) == 0:
        raise HTTPException(status_code=400, detail="Message cannot be empty")
    
    # Validate selected text if in selection mode
    if request.mode == "selection_only":
        if not request.selected_text:
            raise HTTPException(status_code=400, detail="Selected text required in selection_only mode")
        if len(request.selected_text) > 12000:
            raise HTTPException(status_code=400, detail="Selected text too long (max 12000 chars)")
    
    # Initialize session if needed
    if request.session_id not in sessions:
        sessions[request.session_id] = {{
            "messages": [],
            "created_at": datetime.utcnow()
        }}
    
    # Process message with RAG
    # TODO: Integrate with your RAG pipeline (Qdrant + OpenAI)
    if request.selected_text:
        # Selection-only mode: query only about selected text
        rag_context = request.selected_text
        prompt = f"Based on this text: {{request.selected_text[:500]}}...\\n\\nQuestion: {{request.message}}"
    else:
        # Normal mode: query entire book
        rag_context = "full_book"  # Query Qdrant for relevant chunks
        prompt = request.message
    
    # Call OpenAI with RAG context
    response_text = f"Response to: {{request.message}}"  # Replace with actual OpenAI call
    
    # Store in session
    message_id = str(uuid.uuid4())
    sessions[request.session_id]["messages"].append({{
        "id": message_id,
        "message": request.message,
        "response": response_text,
        "selected_text": request.selected_text,
        "timestamp": datetime.utcnow()
    }})
    
    # Store in database if configured
    {"# Save to database" if db_type == "postgres" else ""}
    
    return ChatResponse(
        response=response_text,
        session_id=request.session_id,
        message_id=message_id
    )

{token_endpoint}

@app.get("/health")
async def health_check():
    return {{"status": "healthy", "timestamp": datetime.utcnow()}}

@app.get("/sessions/{{session_id}}")
async def get_session(
    session_id: str,
    {"token: dict = Depends(verify_token)" if auth_type == "better-auth" else ""}
):
    if session_id not in sessions:
        raise HTTPException(status_code=404, detail="Session not found")
    return sessions[session_id]

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
'''


def main():
    parser = argparse.ArgumentParser(description='Generate FastAPI backend template')
    parser.add_argument('--auth', choices=['none', 'better-auth'], default='none')
    parser.add_argument('--db', choices=['memory', 'postgres'], default='memory')
    parser.add_argument('--output', default='./backend.py', help='Output file')
    
    args = parser.parse_args()
    
    backend_code = generate_fastapi_backend(args.auth, args.db)
    
    output_path = Path(args.output)
    output_path.write_text(backend_code)
    
    print(f"✅ Generated FastAPI backend: {output_path}")
    print(f"   Auth: {args.auth}")
    print(f"   Database: {args.db}")
    print(f"\\nNext steps:")
    print(f"1. Install dependencies: pip install fastapi uvicorn sqlalchemy psycopg2-binary")
    if args.auth == "better-auth":
        print(f"2. Install auth: pip install better-auth pyjwt")
    print(f"3. Configure DATABASE_URL environment variable")
    print(f"4. Run: python {output_path}")


if __name__ == '__main__':
    main()