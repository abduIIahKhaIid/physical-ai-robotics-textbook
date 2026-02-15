import React from 'react';
import type { Message } from './ChatProvider';

interface Props {
  message: Message;
}

export function MessageBubble({ message }: Props) {
  const isUser = message.role === 'user';

  return (
    <div className={`message-bubble ${isUser ? 'message-bubble-user' : 'message-bubble-assistant'}`}>
      {isUser && message.selectedText && (
        <div className="message-selected-label">Re: selected text</div>
      )}
      <div>{message.content}</div>
      {!isUser && message.citations && message.citations.length > 0 && (
        <div className="message-citations">
          {message.citations.map((citation, i) => (
            <div key={i}>
              <a href={citation.url} target="_blank" rel="noopener noreferrer">
                {citation.title} &mdash; {citation.section}
              </a>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
