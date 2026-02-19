# Urdu Translation UX Patterns

## RTL (Right-to-Left) Considerations

Urdu is written right-to-left. Key technical requirements:

- Apply `dir="rtl"` attribute to Urdu content containers
- Use `lang="ur"` for proper language identification
- Ensure CSS doesn't break with RTL (avoid hardcoded left/right values)
- Test with Arabic/Urdu fonts: `font-family: 'Noto Nastaliq Urdu', 'Urdu Naskh', Arial, sans-serif`

## Translation Quality Labels

Always label machine-translated content to maintain transparency:

```html
<div class="translation-notice">
  <span>🤖 Machine translated from English</span>
</div>
```

## Caching Strategy

### Option 1: Database Cache (Recommended for Dynamic Content)
Store in Neon Postgres:
```sql
CREATE TABLE translation_cache (
  doc_id TEXT,
  version TEXT,
  lang TEXT,
  content JSONB,
  created_at TIMESTAMP DEFAULT NOW(),
  PRIMARY KEY (doc_id, version, lang)
);
```

### Option 2: File Cache (Better for Static Builds)
Store as markdown files:
```
/translations/ur/module-1/chapter-1.md
/translations/ur/module-2/chapter-1.md
```

## API Endpoint Design

```typescript
POST /api/translate
{
  "doc_id": "module-1-chapter-2",
  "version": "1.0",
  "lang": "ur",
  "content": "Chapter content...",
  "selected_section": null  // optional: translate only selection
}

Response:
{
  "translated": "ترجمہ شدہ مواد...",
  "cached": false,
  "source": "openai-gpt4"
}
```

## UX Mode Comparison

| Mode | Pros | Cons | Best For |
|------|------|------|----------|
| Toggle | Simple UX, single URL | Needs render logic | Most use cases |
| Separate Page | Clean routing, easy caching | More complex i18n | SEO-critical sites |
| Side-by-side | Great for learning | Layout complexity | Educational content |

## Mobile Considerations

- Toggle mode: Stack content vertically
- Side-by-side: Switch to tabs or accordion
- Ensure touch targets are >44px
- Test with mobile Urdu keyboards

## Performance Tips

1. **Lazy load translations**: Don't load until requested
2. **Cache aggressively**: Store in IndexedDB for offline access
3. **Progressive loading**: Translate visible sections first
4. **Debounce requests**: If translating selections, wait 500ms after user stops selecting