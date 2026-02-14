"""Split markdown content into structural blocks for chunking."""

import enum
import re
from dataclasses import dataclass, field


class BlockType(str, enum.Enum):
    HEADING = "heading"
    PROSE = "prose"
    CODE = "code"
    TABLE = "table"
    ADMONITION = "admonition"


@dataclass
class StructuralBlock:
    block_type: BlockType
    text: str
    heading_level: int = 0
    heading_text: str = ""
    lines: list[str] = field(default_factory=list)


def split_markdown(content: str) -> list[StructuralBlock]:
    """Parse markdown into a list of structural blocks.

    Rules:
    - Fenced code blocks (``` or ~~~) are atomic CODE blocks.
    - Pipe tables (lines starting with |) are atomic TABLE blocks.
    - ::: admonitions are atomic ADMONITION blocks.
    - # headings are HEADING blocks with level info.
    - Consecutive prose lines are merged into single PROSE blocks.
    - LaTeX ($$..$$, $...$) is preserved within blocks.
    """
    lines = content.split("\n")
    blocks: list[StructuralBlock] = []
    i = 0

    while i < len(lines):
        line = lines[i]

        # Fenced code block
        fence_match = re.match(r"^(`{3,}|~{3,})", line)
        if fence_match:
            fence = fence_match.group(1)
            fence_char = fence[0]
            fence_len = len(fence)
            code_lines = [line]
            i += 1
            while i < len(lines):
                code_lines.append(lines[i])
                close_match = re.match(
                    rf"^{re.escape(fence_char)}{{{fence_len},}}\s*$", lines[i]
                )
                if close_match:
                    i += 1
                    break
                i += 1
            blocks.append(
                StructuralBlock(
                    block_type=BlockType.CODE,
                    text="\n".join(code_lines),
                    lines=code_lines,
                )
            )
            continue

        # Admonition (:::)
        if line.strip().startswith(":::"):
            admon_lines = [line]
            i += 1
            while i < len(lines):
                admon_lines.append(lines[i])
                if lines[i].strip() == ":::":
                    i += 1
                    break
                i += 1
            blocks.append(
                StructuralBlock(
                    block_type=BlockType.ADMONITION,
                    text="\n".join(admon_lines),
                    lines=admon_lines,
                )
            )
            continue

        # Heading
        heading_match = re.match(r"^(#{1,6})\s+(.+)$", line)
        if heading_match:
            level = len(heading_match.group(1))
            text = heading_match.group(2).strip()
            blocks.append(
                StructuralBlock(
                    block_type=BlockType.HEADING,
                    text=line,
                    heading_level=level,
                    heading_text=text,
                    lines=[line],
                )
            )
            i += 1
            continue

        # Pipe table
        if line.strip().startswith("|"):
            table_lines = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                table_lines.append(lines[i])
                i += 1
            blocks.append(
                StructuralBlock(
                    block_type=BlockType.TABLE,
                    text="\n".join(table_lines),
                    lines=table_lines,
                )
            )
            continue

        # Blank line - skip
        if not line.strip():
            i += 1
            continue

        # Prose - accumulate consecutive non-blank lines
        prose_lines = []
        while i < len(lines):
            current = lines[i]
            if not current.strip():
                i += 1
                break
            if re.match(r"^(#{1,6})\s+", current):
                break
            if re.match(r"^(`{3,}|~{3,})", current):
                break
            if current.strip().startswith("|"):
                break
            if current.strip().startswith(":::"):
                break
            prose_lines.append(current)
            i += 1
        if prose_lines:
            blocks.append(
                StructuralBlock(
                    block_type=BlockType.PROSE,
                    text="\n".join(prose_lines),
                    lines=prose_lines,
                )
            )

    return blocks
