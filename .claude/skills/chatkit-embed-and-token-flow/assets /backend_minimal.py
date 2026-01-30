"""
Minimal FastAPI backend for ChatKit integration
Copy this as a starting point and customize for your needs
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import uuid
from datetime import datetime

app = FastAPI(title="ChatKit Backend")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Replace with your GitHub Pages URL in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# In-memory session storage (use database in production)
sessions = {}

class ChatRequest(BaseModel):
    message: str
    session_id: str
    mode: str = "normal"
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    session_id: str
    message_id: str

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    # Validate request
    if not request.message.strip():
        raise HTTPException(status_code=400, detail="Message cannot be empty")
    
    if request.mode == "selection_only" and not request.selected_text:
        raise HTTPException(status_code=400, detail="Selected text required in selection_only mode")
    
    if request.selected_text and len(request.selected_text) > 12000:
        raise HTTPException(status_code=400, detail="Selected text too long (max 12000 chars)")
    
    # Initialize session if needed
    if request.session_id not in sessions:
        sessions[request.session_id] = {
            "messages": [],
            "created_at": datetime.utcnow()
        }
    
    # TODO: Integrate with your RAG pipeline
    # Example:
    # if request.selected_text:
    #     context = request.selected_text
    # else:
    #     context = query_qdrant(request.message)
    #
    # response_text = call_openai(request.message, context)
    
    # Placeholder response
    if request.selected_text:
        response_text = f"Based on the selected text, here's my answer to '{request.message}': [Your RAG response here]"
    else:
        response_text = f"Answer to '{request.message}': [Your RAG response here]"
    
    # Store message
    message_id = str(uuid.uuid4())
    sessions[request.session_id]["messages"].append({
        "id": message_id,
        "message": request.message,
        "response": response_text,
        "selected_text": request.selected_text,
        "timestamp": datetime.utcnow()
    })
    
    return ChatResponse(
        response=response_text,
        session_id=request.session_id,
        message_id=message_id
    )

@app.get("/health")
async def health_check():
    return {"status": "healthy", "sessions": len(sessions)}

@app.get("/sessions/{session_id}")
async def get_session(session_id: str):
    if session_id not in sessions:
        raise HTTPException(status_code=404, detail="Session not found")
    return sessions[session_id]

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)