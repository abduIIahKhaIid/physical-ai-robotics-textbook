"""Convert QueryFilters to Qdrant filter objects."""

from qdrant_client.models import FieldCondition, Filter, MatchAny

from rag.models import QueryFilters


def build_qdrant_filter(filters: QueryFilters | None) -> Filter | None:
    """Convert QueryFilters into a Qdrant Filter.

    Returns None when no filters are active (empty/None).
    """
    if filters is None:
        return None

    conditions: list[FieldCondition] = []

    if filters.modules:
        conditions.append(
            FieldCondition(key="module", match=MatchAny(any=filters.modules))
        )
    if filters.chapters:
        conditions.append(
            FieldCondition(key="chapter", match=MatchAny(any=filters.chapters))
        )
    if filters.content_types:
        conditions.append(
            FieldCondition(
                key="content_type", match=MatchAny(any=filters.content_types)
            )
        )
    if filters.tags:
        conditions.append(
            FieldCondition(key="tags", match=MatchAny(any=filters.tags))
        )

    if not conditions:
        return None

    return Filter(must=conditions)
