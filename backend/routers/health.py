"""Health check endpoint: GET /api/v1/health."""

from __future__ import annotations

import logging
from datetime import datetime, timezone

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from qdrant_client import QdrantClient

from backend.models.api_models import HealthComponent, HealthResponse
from rag.config import load_settings as load_rag_settings

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/health")
async def health_check(request: Request) -> JSONResponse:
    """Check backend and dependency health."""
    components: dict[str, HealthComponent] = {}
    all_healthy = True

    # Check database
    pool = getattr(request.app.state, "db_pool", None)
    if pool is not None:
        try:
            await pool.execute("SELECT 1")
            components["database"] = HealthComponent(status="ok")
        except Exception as exc:
            logger.warning(f"Database health check failed: {exc}")
            components["database"] = HealthComponent(
                status="error", message="Connection failed"
            )
            all_healthy = False
    else:
        components["database"] = HealthComponent(
            status="error", message="Not configured"
        )
        all_healthy = False

    # Check vector store (Qdrant)
    try:
        rag_settings = load_rag_settings()
        qdrant = QdrantClient(
            url=rag_settings.qdrant_url,
            api_key=rag_settings.qdrant_api_key or None,
            timeout=5,
        )
        qdrant.get_collections()
        components["vector_store"] = HealthComponent(status="ok")
    except Exception as exc:
        logger.warning(f"Vector store health check failed: {exc}")
        components["vector_store"] = HealthComponent(
            status="error", message="Connection failed"
        )
        all_healthy = False

    status = "healthy" if all_healthy else "degraded"
    status_code = 200 if all_healthy else 503

    response = HealthResponse(
        status=status,
        timestamp=datetime.now(timezone.utc),
        components=components,
    )

    return JSONResponse(
        status_code=status_code,
        content=response.model_dump(mode="json"),
    )
