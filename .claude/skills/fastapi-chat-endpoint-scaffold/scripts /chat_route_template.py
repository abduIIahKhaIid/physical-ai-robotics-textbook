"""
Chat Route Template

Implements the /chat endpoint with:
- Normal RAG mode (Qdrant retrieval)
- Selection-only mode (no retrieval)
- Session management
- Streaming support (optional)

This template demonstrates the critical contract for handling selected_text.
"""

from fastapi import APIRouter, HTTPException, Depends
from fastapi.responses import StreamingResponse
import uuid
import logging
from typing import Optional

from app.models.schemas import ChatRequest, ChatResponse
from app.services.database import get_session, create_session, store_message
from app.services.rag import retrieve_context
from app.services.chat import generate_response, generate_streaming_response

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Main chat endpoint supporting both RAG and selection-only modes.
    
    Mode determination:
    - If selected_text is provided → selection_only mode (skip RAG)
    - Otherwise → normal mode (use RAG with Qdrant)
    """
    
    # 1. Session Management
    session_id = request.session_id or str(uuid.uuid4())
    
    try:
        session = await get_session(session_id)
        if not session:
            session = await create_session(session_id)
            logger.info(f"Created new session: {session_id}")
    except Exception as e:
        logger.error(f"Session creation failed: {e}")
        raise HTTPException(status_code=500, detail="Session creation failed")
    
    # 2. Store user message
    await store_message(
        session_id=session_id,
        role="user",
        content=request.message,
        selected_text=request.selected_text
    )
    
    # 3. Determine mode and process
    if request.selected_text:
        # SELECTION-ONLY MODE
        logger.info(f"Using selection-only mode for session {session_id}")
        mode = "selection_only"
        
        # Skip Qdrant retrieval entirely
        context_chunks = []
        citations = []
        
        # Generate response using only selected text
        try:
            reply = await generate_response(
                message=request.message,
                context="",  # No RAG context
                selected_text=request.selected_text,
                lang=request.lang,
                mode="selection_only"
            )
        except Exception as e:
            logger.error(f"Selection-only response generation failed: {e}")
            raise HTTPException(
                status_code=500,
                detail="Failed to generate response from selected text"
            )
    
    else:
        # NORMAL RAG MODE
        logger.info(f"Using normal RAG mode for session {session_id}")
        mode = "normal"
        
        # Retrieve context from Qdrant
        try:
            context_chunks = await retrieve_context(
                query=request.message,
                top_k=5,
                lang=request.lang
            )
            
            if not context_chunks:
                logger.warning(f"No context retrieved for query: {request.message}")
                context = ""
                citations = []
            else:
                context = "\n\n".join([chunk["text"] for chunk in context_chunks])
                citations = [
                    {
                        "text": chunk["text"],
                        "source": chunk["metadata"].get("source", "Unknown"),
                        "score": chunk["score"]
                    }
                    for chunk in context_chunks
                ]
        except Exception as e:
            logger.error(f"RAG retrieval failed: {e}")
            # Graceful degradation - continue without context
            context = ""
            citations = []
        
        # Generate response with RAG context
        try:
            reply = await generate_response(
                message=request.message,
                context=context,
                selected_text=None,
                lang=request.lang,
                mode="normal"
            )
        except Exception as e:
            logger.error(f"Response generation failed: {e}")
            raise HTTPException(
                status_code=500,
                detail="Failed to generate response"
            )
    
    # 4. Store assistant reply
    await store_message(
        session_id=session_id,
        role="assistant",
        content=reply
    )
    
    # 5. Return response
    return ChatResponse(
        session_id=session_id,
        reply=reply,
        citations=citations if mode == "normal" else None,
        used_mode=mode
    )


@router.post("/chat/stream")
async def chat_streaming(request: ChatRequest):
    """
    Streaming variant of chat endpoint using Server-Sent Events.
    
    Returns tokens as they are generated for a more responsive UX.
    """
    
    session_id = request.session_id or str(uuid.uuid4())
    
    # Session management
    session = await get_session(session_id)
    if not session:
        session = await create_session(session_id)
    
    await store_message(
        session_id=session_id,
        role="user",
        content=request.message,
        selected_text=request.selected_text
    )
    
    # Determine mode
    if request.selected_text:
        mode = "selection_only"
        context = ""
    else:
        mode = "normal"
        context_chunks = await retrieve_context(request.message, top_k=5)
        context = "\n\n".join([c["text"] for c in context_chunks]) if context_chunks else ""
    
    # Stream response
    async def event_stream():
        full_reply = ""
        
        async for token in generate_streaming_response(
            message=request.message,
            context=context,
            selected_text=request.selected_text,
            lang=request.lang,
            mode=mode
        ):
            full_reply += token
            yield f"data: {token}\n\n"
        
        # Store complete reply
        await store_message(
            session_id=session_id,
            role="assistant",
            content=full_reply
        )
        
        yield "data: [DONE]\n\n"
    
    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream"
    )