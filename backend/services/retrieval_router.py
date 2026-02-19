"""Lightweight LLM-based classifier to decide if retrieval is needed."""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

logger = logging.getLogger(__name__)

_CLASSIFICATION_PROMPT = """\
Classify whether this student message needs a textbook search.

Reply with a single word: RETRIEVE or SKIP.

RETRIEVE: questions about course content (robotics, ROS2, simulation, kinematics, etc.)
SKIP: greetings, thanks, follow-up questions that reference the conversation, meta questions about the assistant

Recent conversation:
{history}

Student message: {message}

Classification:"""


def _format_history(history: list[dict], max_turns: int = 2) -> str:
    """Format the last N history turns for the classification prompt."""
    recent = history[-max_turns * 2 :] if history else []
    if not recent:
        return "(no prior messages)"
    lines = []
    for msg in recent:
        role = "Student" if msg.get("role") == "user" else "Assistant"
        lines.append(f"{role}: {msg.get('content', '')}")
    return "\n".join(lines)


async def should_retrieve(
    message: str,
    history: list[dict],
    gemini_client: Any,
    model: str,
    timeout_s: int = 5,
) -> bool:
    """Classify whether a message needs textbook retrieval.

    Returns True (retrieve) on any ambiguity or error — safe default.
    """
    start = time.monotonic()
    prompt = _CLASSIFICATION_PROMPT.format(
        history=_format_history(history),
        message=message,
    )

    try:
        response = await asyncio.wait_for(
            asyncio.to_thread(
                gemini_client.models.generate_content,
                model=model,
                contents=[prompt],
            ),
            timeout=timeout_s,
        )
        raw = (response.text or "").strip().split()[0].upper()
        decision = raw != "SKIP"
    except asyncio.TimeoutError:
        logger.warning(
            "Router classification timed out after %ds, defaulting to retrieve",
            timeout_s,
        )
        decision = True
    except Exception:
        logger.warning("Router classification failed, defaulting to retrieve")
        decision = True

    latency = time.monotonic() - start
    label = "RETRIEVE" if decision else "SKIP"
    logger.info(
        'Router decision: %s for "%s" (latency: %.2fs)',
        label,
        message[:80],
        latency,
    )
    return decision
