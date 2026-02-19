/**
 * Fetch-based SSE client for POST requests to the chat backend.
 * Native EventSource only supports GET, so we use fetch + ReadableStream.
 */

export interface Citation {
  title: string;
  section: string;
  url: string;
  module: string;
  chapter: string;
}

export interface ChatRequest {
  message: string;
  session_id?: string | null;
  mode: 'normal' | 'selected_text_only';
  selected_text?: string | null;
  source_doc_path?: string | null;
  source_section?: string | null;
}

export type SSEEvent =
  | { type: 'token'; content: string }
  | { type: 'done'; sessionId: string; messageId: string; citations: Citation[]; persistenceWarning?: boolean }
  | { type: 'error'; code: string; message: string }
  | { type: 'rate_limited'; retryAfter: number }
  | { type: 'session_not_found' };

/**
 * Send a chat request via POST and yield SSE events as they arrive.
 */
export async function* postChat(
  backendUrl: string,
  request: ChatRequest,
): AsyncGenerator<SSEEvent> {
  let response: Response;

  try {
    response = await fetch(`${backendUrl}/api/v1/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'text/event-stream',
      },
      body: JSON.stringify(request),
    });
  } catch {
    yield { type: 'error', code: 'network_error', message: 'Cannot reach server. Please check your connection.' };
    return;
  }

  // Handle HTTP-level errors before reading the stream
  if (response.status === 429) {
    const retryAfter = parseInt(response.headers.get('Retry-After') || '60', 10);
    yield { type: 'rate_limited', retryAfter };
    return;
  }

  if (response.status === 404) {
    yield { type: 'session_not_found' };
    return;
  }

  if (!response.ok) {
    let errorMessage = response.statusText || 'Request failed';
    try {
      const errorBody = await response.json();
      if (errorBody?.error?.message) {
        errorMessage = errorBody.error.message;
      }
    } catch {
      // Ignore JSON parse errors
    }
    yield { type: 'error', code: `http_${response.status}`, message: errorMessage };
    return;
  }

  // Read the SSE stream
  const reader = response.body?.getReader();
  if (!reader) {
    yield { type: 'error', code: 'no_stream', message: 'Response body is not readable' };
    return;
  }

  const decoder = new TextDecoder();
  let buffer = '';

  try {
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });

      // SSE events are separated by double newlines (\n\n or \r\n\r\n)
      // Normalize \r\n to \n so the split works with either format
      buffer = buffer.replace(/\r\n/g, '\n');
      const parts = buffer.split('\n\n');
      // Keep the last incomplete part in the buffer
      buffer = parts.pop() || '';

      for (const part of parts) {
        const event = parseSSEEvent(part);
        if (event) yield event;
      }
    }

    // Process any remaining buffer
    if (buffer.trim()) {
      const event = parseSSEEvent(buffer);
      if (event) yield event;
    }
  } catch {
    yield { type: 'error', code: 'stream_error', message: 'Connection interrupted' };
  } finally {
    reader.releaseLock();
  }
}

/**
 * Parse a single SSE event block into a typed SSEEvent.
 */
function parseSSEEvent(raw: string): SSEEvent | null {
  let eventType = '';
  let data = '';

  for (const line of raw.split('\n')) {
    if (line.startsWith('event:')) {
      eventType = line.slice(6).trim();
    } else if (line.startsWith('data:')) {
      data = line.slice(5).trim();
    }
  }

  if (!eventType || !data) return null;

  try {
    const parsed = JSON.parse(data);

    switch (eventType) {
      case 'token':
        return { type: 'token', content: parsed.content || '' };
      case 'done':
        return {
          type: 'done',
          sessionId: parsed.session_id,
          messageId: parsed.message_id,
          citations: parsed.citations || [],
          persistenceWarning: parsed.persistence_warning || false,
        };
      case 'error':
        return { type: 'error', code: parsed.code || 'unknown', message: parsed.message || 'Unknown error' };
      default:
        return null;
    }
  } catch {
    return null;
  }
}
