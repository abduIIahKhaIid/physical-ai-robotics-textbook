"""Grounding policy enforcement for retrieval results."""

from rag.models import GroundedResponse, RetrievalResult
from rag.response.citation import format_context_with_citations


_REFUSAL_INSTRUCTION = (
    "The textbook excerpts do not contain sufficient information "
    "to answer this question. Please tell the user that you cannot "
    "find relevant information in the textbook for their query."
)

_SELECTED_TEXT_INSTRUCTION = (
    "Answer ONLY from the following selected text. "
    "Do not use any other knowledge. "
    "If the text does not contain enough information to answer, say so."
)

_NORMAL_INSTRUCTION = (
    "Answer based on the following textbook excerpts. "
    "Cite the source for each fact. "
    "If the excerpts don't contain enough information, explicitly state that."
)


def apply_grounding_policy(
    result: RetrievalResult,
    question: str,
) -> GroundedResponse:
    """Apply groundedness constraints to a retrieval result.

    Rules:
    - No chunks → insufficient context, refusal instruction.
    - selected_text_only mode → strict instruction (no outside knowledge).
    - normal mode → citation-based instruction.
    """
    citations = [chunk.citation for chunk in result.chunks]

    if not result.chunks:
        return GroundedResponse(
            context="",
            system_instruction=_REFUSAL_INSTRUCTION,
            citations=[],
            mode=result.mode,
            sufficient_context=False,
        )

    context = format_context_with_citations(result.chunks)

    if result.mode == "selected_text_only":
        instruction = _SELECTED_TEXT_INSTRUCTION
    else:
        instruction = _NORMAL_INSTRUCTION

    return GroundedResponse(
        context=context,
        system_instruction=instruction,
        citations=citations,
        mode=result.mode,
        sufficient_context=True,
    )
