"""
Example: FastAPI Translation Endpoint
Place in: backend/translate.py or api/translate.py

This endpoint handles translation requests and caching.
"""

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Optional
import os
import hashlib
import json
from datetime import datetime

# Assuming you have these imports configured
# from your_db import get_db_connection
# from openai import OpenAI

app = FastAPI()

class TranslationRequest(BaseModel):
    doc_id: str
    version: str = "1.0"
    lang: str = "ur"
    content: str
    selected_section: Optional[str] = None


class TranslationResponse(BaseModel):
    translated: str
    cached: bool
    source: str
    created_at: str


def get_cache_key(doc_id: str, version: str, lang: str) -> str:
    """Generate cache key for translation"""
    return f"translation_{doc_id}_{version}_{lang}"


async def get_cached_translation(doc_id: str, version: str, lang: str):
    """
    Retrieve cached translation from database
    
    Example SQL:
    SELECT content FROM translation_cache 
    WHERE doc_id = ? AND version = ? AND lang = ?
    """
    # TODO: Implement database lookup
    # conn = get_db_connection()
    # result = conn.execute(
    #     "SELECT content, created_at FROM translation_cache WHERE doc_id = ? AND version = ? AND lang = ?",
    #     (doc_id, version, lang)
    # ).fetchone()
    # return result if result else None
    return None


async def save_translation_cache(doc_id: str, version: str, lang: str, content: str):
    """
    Save translation to database cache
    
    Example SQL:
    INSERT INTO translation_cache (doc_id, version, lang, content, created_at)
    VALUES (?, ?, ?, ?, NOW())
    ON CONFLICT (doc_id, version, lang) DO UPDATE SET content = ?, created_at = NOW()
    """
    # TODO: Implement database storage
    pass


async def translate_with_openai(content: str, target_lang: str = "ur") -> str:
    """
    Translate content using OpenAI API
    
    Note: Adjust prompt based on whether content is technical documentation
    """
    from openai import OpenAI
    
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    
    prompt = f"""Translate the following technical documentation to {target_lang} (Urdu).
Maintain:
- Technical terms (e.g., "API", "database", "framework") in English or use accepted Urdu equivalents
- Code blocks exactly as-is
- Markdown formatting
- Professional, educational tone

Content to translate:
{content}
"""
    
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a technical translator specializing in educational content."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.3
    )
    
    return response.choices[0].message.content


@app.post("/api/translate", response_model=TranslationResponse)
async def translate_content(request: TranslationRequest):
    """
    Translate content to target language with caching
    """
    try:
        # Check cache first
        cached = await get_cached_translation(request.doc_id, request.version, request.lang)
        
        if cached:
            return TranslationResponse(
                translated=cached['content'],
                cached=True,
                source="cache",
                created_at=cached['created_at']
            )
        
        # Translate with OpenAI
        content_to_translate = request.selected_section if request.selected_section else request.content
        translated = await translate_with_openai(content_to_translate, request.lang)
        
        # Save to cache
        await save_translation_cache(request.doc_id, request.version, request.lang, translated)
        
        return TranslationResponse(
            translated=translated,
            cached=False,
            source="openai-gpt4",
            created_at=datetime.now().isoformat()
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")


@app.delete("/api/translate/cache/{doc_id}")
async def clear_translation_cache(doc_id: str, version: str = "1.0", lang: str = "ur"):
    """
    Clear cached translation for a specific document
    Useful for when content is updated
    """
    # TODO: Implement cache clearing
    # conn = get_db_connection()
    # conn.execute("DELETE FROM translation_cache WHERE doc_id = ? AND version = ? AND lang = ?", 
    #              (doc_id, version, lang))
    return {"message": "Cache cleared", "doc_id": doc_id}