import React from 'react';
import { useChatContext } from './ChatProvider';

export function SelectedTextChip() {
  const { selectedText, setSelectedText } = useChatContext();

  if (!selectedText) return null;

  const preview = selectedText.text.length > 120
    ? selectedText.text.slice(0, 120) + '...'
    : selectedText.text;

  return (
    <div className="selected-text-chip">
      <span className="selected-text-chip-text">&ldquo;{preview}&rdquo;</span>
      <button
        className="selected-text-chip-dismiss"
        onClick={() => setSelectedText(null)}
        aria-label="Remove selected text"
      >
        &times;
      </button>
    </div>
  );
}
