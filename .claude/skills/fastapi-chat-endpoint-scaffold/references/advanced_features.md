# Advanced Features Reference

## Better Auth Integration

### Setup Better Auth

Install dependencies:
```bash
npm install better-auth
```

### Backend Integration

Add auth middleware to FastAPI:

```python
# app/middleware/auth.py
from fastapi import HTTPException, Header
import jwt

async def verify_token(authorization: str = Header(None)):
    if not authorization:
        raise HTTPException(status_code=401, detail="Missing authorization header")
    
    try:
        token = authorization.replace("Bearer ", "")
        payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")
```

### User Background Questionnaire

**Database schema:**
```sql
CREATE TABLE user_profiles (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(255) UNIQUE NOT NULL,
    software_background VARCHAR(50),  -- 'beginner', 'intermediate', 'advanced'
    hardware_experience VARCHAR(50),  -- 'none', 'hobbyist', 'professional'
    programming_languages TEXT[],
    robotics_experience BOOLEAN,
    created_at TIMESTAMP,
    updated_at TIMESTAMP
);
```

**Signup endpoint:**
```python
@router.post("/auth/signup")
async def signup(
    email: str,
    password: str,
    software_background: str,
    hardware_experience: str,
    programming_languages: List[str]
):
    # Create user with Better Auth
    user = await create_user(email, password)
    
    # Store profile
    await db.execute("""
        INSERT INTO user_profiles 
        (user_id, software_background, hardware_experience, programming_languages)
        VALUES ($1, $2, $3, $4)
    """, user.id, software_background, hardware_experience, programming_languages)
    
    return {"user_id": user.id, "token": generate_token(user)}
```

## Content Personalization

### Dynamic Content Adjustment

**Strategy:** Adjust technical depth based on user profile

```python
async def personalize_content(user_id: str, content: str) -> str:
    profile = await get_user_profile(user_id)
    
    if profile.software_background == "beginner":
        # Add more explanations, analogies
        system_prompt = """
        Explain technical concepts using simple analogies.
        Avoid jargon. Define all technical terms.
        """
    elif profile.software_background == "advanced":
        # More technical, code-heavy
        system_prompt = """
        Be concise and technical. Include code examples.
        Assume knowledge of CS fundamentals.
        """
    
    # Call LLM with personalized system prompt
    response = await openai_client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Personalize this: {content}"}
        ]
    )
    
    return response.choices[0].message.content
```

### Chapter-Level Personalization

**Frontend button integration:**
```typescript
// In Docusaurus MDX
<PersonalizeButton chapterContent={mdxContent} />
```

**Backend endpoint:**
```python
@router.post("/personalize")
async def personalize_chapter(
    user_id: str,
    chapter_content: str,
    current_user: dict = Depends(verify_token)
):
    profile = await get_user_profile(user_id)
    personalized = await personalize_content(user_id, chapter_content)
    
    return {"personalized_content": personalized}
```

## Urdu Translation

### Translation Endpoint

```python
@router.post("/translate")
async def translate_to_urdu(
    content: str,
    preserve_code: bool = True
):
    if preserve_code:
        # Extract code blocks
        code_blocks = extract_code_blocks(content)
        text_only = remove_code_blocks(content)
        
        # Translate text
        translated_text = await openai_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {
                    "role": "system",
                    "content": "Translate to Urdu. Keep technical terms in English."
                },
                {"role": "user", "content": text_only}
            ]
        )
        
        # Recombine with code blocks
        final_content = reinsert_code_blocks(
            translated_text.choices[0].message.content,
            code_blocks
        )
    else:
        final_content = await translate_full(content)
    
    return {"translated_content": final_content}
```

### Frontend Integration

```typescript
const TranslateButton = ({ content }) => {
  const [translated, setTranslated] = useState(null);
  
  const handleTranslate = async () => {
    const response = await fetch('/api/translate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ content })
    });
    
    const data = await response.json();
    setTranslated(data.translated_content);
  };
  
  return (
    <div>
      <button onClick={handleTranslate}>Translate to Urdu</button>
      {translated && <div dangerouslySetInnerHTML={{ __html: translated }} />}
    </div>
  );
};
```

## Rate Limiting

### Using SlowAPI

```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.post("/chat")
@limiter.limit("10/minute")
async def chat_endpoint(request: Request, chat_req: ChatRequest):
    # ... chat logic
    pass
```

## Caching with Redis

### Setup Redis Client

```python
import redis.asyncio as redis

redis_client = redis.from_url("redis://localhost:6379")

async def get_cached_response(session_id: str, message: str):
    cache_key = f"chat:{session_id}:{hash(message)}"
    cached = await redis_client.get(cache_key)
    
    if cached:
        return json.loads(cached)
    
    return None

async def cache_response(session_id: str, message: str, response: dict):
    cache_key = f"chat:{session_id}:{hash(message)}"
    await redis_client.setex(
        cache_key,
        3600,  # 1 hour TTL
        json.dumps(response)
    )
```

## Observability

### Logging Best Practices

```python
import structlog

logger = structlog.get_logger()

@app.post("/chat")
async def chat_endpoint(chat_req: ChatRequest):
    logger.info(
        "chat_request_received",
        session_id=chat_req.session_id,
        message_length=len(chat_req.message),
        mode=chat_req.mode,
        has_selection=bool(chat_req.selected_text)
    )
    
    try:
        response = await process_chat(chat_req)
        
        logger.info(
            "chat_response_sent",
            session_id=response.session_id,
            reply_length=len(response.reply),
            citations_count=len(response.citations or [])
        )
        
        return response
    except Exception as e:
        logger.error(
            "chat_error",
            session_id=chat_req.session_id,
            error=str(e),
            exc_info=True
        )
        raise
```

### Metrics with Prometheus

```python
from prometheus_client import Counter, Histogram

chat_requests = Counter('chat_requests_total', 'Total chat requests')
chat_duration = Histogram('chat_duration_seconds', 'Chat request duration')

@app.post("/chat")
async def chat_endpoint(chat_req: ChatRequest):
    chat_requests.inc()
    
    with chat_duration.time():
        response = await process_chat(chat_req)
    
    return response
```