import React, { useEffect, useState, useCallback, useRef } from 'react';
import { useChatContext } from './ChatProvider';
import { MIN_SELECTION_LENGTH } from '../../utils/chatConfig';

interface TooltipPosition {
  top: number;
  left: number;
}

export function SelectionTooltip() {
  const { setSelectedText, togglePanel, isOpen } = useChatContext();
  const [visible, setVisible] = useState(false);
  const [position, setPosition] = useState<TooltipPosition>({ top: 0, left: 0 });
  const selectedTextRef = useRef('');
  const tooltipRef = useRef<HTMLButtonElement>(null);

  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    if (!selection || selection.isCollapsed) {
      setVisible(false);
      return;
    }

    const text = selection.toString().trim();
    if (text.length < MIN_SELECTION_LENGTH) {
      setVisible(false);
      return;
    }

    // Don't show tooltip if selection is inside the chat widget
    const anchorNode = selection.anchorNode;
    if (anchorNode) {
      const parent = anchorNode.parentElement;
      if (parent?.closest('.chat-panel') || parent?.closest('.chat-button')) {
        setVisible(false);
        return;
      }
    }

    try {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      setPosition({
        top: rect.top + window.scrollY - 40,
        left: rect.left + rect.width / 2,
      });
      selectedTextRef.current = text;
      setVisible(true);
    } catch {
      setVisible(false);
    }
  }, []);

  const handleClick = useCallback(() => {
    setSelectedText({
      text: selectedTextRef.current,
      pageUrl: window.location.pathname,
      pageTitle: document.title,
    });

    if (!isOpen) {
      togglePanel();
    }

    // Clear browser selection
    window.getSelection()?.removeAllRanges();
    setVisible(false);
  }, [setSelectedText, togglePanel, isOpen]);

  useEffect(() => {
    const handleMouseUp = () => {
      // Delay to let selection settle
      setTimeout(handleSelection, 10);
    };

    const handleSelectionChange = () => {
      const selection = window.getSelection();
      if (!selection || selection.isCollapsed) {
        setVisible(false);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('touchend', handleMouseUp);
    document.addEventListener('selectionchange', handleSelectionChange);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('touchend', handleMouseUp);
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, [handleSelection]);

  if (!visible) return null;

  return (
    <button
      ref={tooltipRef}
      className="selection-tooltip"
      style={{
        top: position.top,
        left: position.left,
      }}
      onClick={handleClick}
      role="tooltip"
    >
      Ask about this
    </button>
  );
}
