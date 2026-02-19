#!/usr/bin/env python3
"""
Generate ChatKit widget integration code for Docusaurus.
Handles both normal and selection-only modes.
"""

import argparse
import json
from pathlib import Path


def generate_widget_component(backend_url: str, mode: str = "normal", auth_type: str = "none") -> str:
    """Generate React component for ChatKit widget."""
    
    return f'''import React, {{ useEffect, useState }} from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const ChatKitWidget = () => {{
  const [isLoaded, setIsLoaded] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {{
    if (!ExecutionEnvironment.canUseDOM) return;

    // Initialize session
    const initSession = () => {{
      let sid = localStorage.getItem('chatkit_session_id');
      if (!sid) {{
        sid = 'session_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
        localStorage.setItem('chatkit_session_id', sid);
      }}
      setSessionId(sid);
    }};

    initSession();

    // Load ChatKit script
    const script = document.createElement('script');
    script.src = 'https://cdn.jsdelivr.net/npm/@openai/chatkit@latest/dist/chatkit.min.js';
    script.async = true;
    script.onload = () => {{
      setIsLoaded(true);
      initializeChatKit();
    }};
    document.body.appendChild(script);

    return () => {{
      if (document.body.contains(script)) {{
        document.body.removeChild(script);
      }}
    }};
  }}, []);

  const initializeChatKit = () => {{
    if (typeof window.ChatKit === 'undefined') return;

    const config = {{
      apiUrl: '{backend_url}',
      mode: '{mode}',
      onMessage: async (message) => {{
        const payload = {{
          message: message.text,
          session_id: sessionId,
          mode: '{mode}',
          ...(selectedText && {{ selected_text: selectedText }})
        }};

        {"const token = await fetchToken();" if auth_type == "better-auth" else "// No auth"}
        
        const response = await fetch('{backend_url}/chat', {{
          method: 'POST',
          headers: {{
            'Content-Type': 'application/json',
            {"'Authorization': `Bearer ${{token}}`," if auth_type == "better-auth" else ""}
          }},
          body: JSON.stringify(payload)
        }});

        return await response.json();
      }}
    }};

    window.ChatKit.init(config);
  }};

  {"const fetchToken = async () => {" if auth_type == "better-auth" else ""}
  {"  const response = await fetch('" + backend_url + "/auth/token');" if auth_type == "better-auth" else ""}
  {"  const data = await response.json();" if auth_type == "better-auth" else ""}
  {"  return data.token;" if auth_type == "better-auth" else ""}
  {"};" if auth_type == "better-auth" else ""}

  // Selection capture
  useEffect(() => {{
    if (!ExecutionEnvironment.canUseDOM || '{mode}' !== 'selection_only') return;

    const handleSelection = () => {{
      const selection = window.getSelection();
      const text = selection.toString().trim();
      if (text.length > 0 && text.length < 12000) {{
        setSelectedText(text);
      }}
    }};

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {{
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    }};
  }}, []);

  return (
    <div id="chatkit-widget-container">
      {{!isLoaded && <div>Loading ChatKit...</div>}}
    </div>
  );
}};

export default ChatKitWidget;
'''


def generate_docusaurus_config_snippet(base_url: str) -> str:
    """Generate docusaurus.config.js snippet for widget integration."""
    
    return f'''// Add to docusaurus.config.js

module.exports = {{
  // ... existing config
  
  baseUrl: '{base_url}',
  
  clientModules: [
    require.resolve('./src/components/ChatKitWidget.js'),
  ],
  
  plugins: [
    // ... existing plugins
  ],
  
  themeConfig: {{
    // ... existing theme config
  }},
}};
'''


def generate_layout_wrapper() -> str:
    """Generate layout wrapper to inject widget globally."""
    
    return '''import React from 'react';
import Layout from '@theme-original/Layout';
import ChatKitWidget from '@site/src/components/ChatKitWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatKitWidget />
    </>
  );
}
'''


def main():
    parser = argparse.ArgumentParser(description='Generate ChatKit widget code')
    parser.add_argument('--backend-url', required=True, help='FastAPI backend URL')
    parser.add_argument('--mode', choices=['normal', 'selection_only'], default='normal')
    parser.add_argument('--auth', choices=['none', 'better-auth'], default='none')
    parser.add_argument('--output-dir', default='./generated', help='Output directory')
    parser.add_argument('--base-url', default='/', help='Docusaurus baseUrl')
    
    args = parser.parse_args()
    
    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Generate component
    widget_code = generate_widget_component(args.backend_url, args.mode, args.auth)
    (output_path / 'ChatKitWidget.js').write_text(widget_code)
    
    # Generate config snippet
    config_code = generate_docusaurus_config_snippet(args.base_url)
    (output_path / 'docusaurus-config-snippet.js').write_text(config_code)
    
    # Generate layout wrapper
    layout_code = generate_layout_wrapper()
    (output_path / 'Layout.js').write_text(layout_code)
    
    print(f"✅ Generated ChatKit widget files in {output_path}")
    print(f"   Mode: {args.mode}")
    print(f"   Auth: {args.auth}")
    print(f"\\nNext steps:")
    print(f"1. Copy ChatKitWidget.js to src/components/")
    print(f"2. Copy Layout.js to src/theme/Layout/")
    print(f"3. Update docusaurus.config.js with snippet from docusaurus-config-snippet.js")


if __name__ == '__main__':
    main()