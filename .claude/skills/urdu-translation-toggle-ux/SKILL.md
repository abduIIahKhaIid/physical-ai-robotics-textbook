---
name: urdu-translation-toggle-ux
description: Implement a stable UX pattern for translating Docusaurus book chapters into Urdu with toggle/button control. Use when implementing Spec 015 (Urdu translation feature), designing translation UI components, setting up translation APIs, or adding language switching to documentation sites. Handles RTL rendering, caching, OpenAI API integration, and user state management without breaking navigation or SEO.
---

# Urdu Translation Toggle UX

Implement a reliable, user-controlled Urdu translation experience for documentation chapters. This skill provides patterns, components, and validation tools for adding translation functionality to Docusaurus-based books.

## Quick Start

```text
/urdu-translation-toggle-ux
```

Provide the chapter path to add Urdu translation toggle. The skill produces a React component with RTL support, caching, and API integration.

## Core Implementation Workflow

The translation feature requires coordination across multiple layers:

1. **Frontend Component**: Button/toggle for user control
2. **Translation API**: Backend endpoint that translates content via OpenAI
3. **Caching Layer**: Store translations to avoid repeated API costs
4. **RTL Styling**: Ensure Urdu renders correctly right-to-left
5. **State Management**: Track language preference per chapter

Follow these steps in order to implement the feature correctly.

## Step 1: Choose Your UX Mode

Select the pattern that best fits your project:

**Mode A: Toggle (Recommended)**
- Single button switches content in-place (English ↔ Urdu)
- Same URL, no routing changes
- Best for: Most documentation sites

**Mode B: Separate Page Route**
- Create `/ur/` routes parallel to English routes
- Example: `/module-1/intro` and `/ur/module-1/intro`
- Best for: SEO-critical sites, pre-generated translations

**Mode C: Side-by-Side**
- Show English and Urdu simultaneously
- Best for: Language learning contexts

For this skill, we focus on **Mode A (Toggle)** as it provides the best balance of simplicity and user experience.

## Step 2: Add the Translation Button Component

Create the translation toggle button that will appear on every chapter page.

**Location**: Copy `assets/TranslationToggle.tsx` to your project's `src/components/` directory.

**Integration**: Add the component to your chapter template. For Docusaurus, this typically means:

1. Create or modify `src/theme/DocItem/Layout/index.tsx`
2. Import the TranslationToggle component
3. Render it in the appropriate location (usually near the chapter title)

```tsx
import TranslationToggle from '@site/src/components/TranslationToggle';

// Inside your chapter render:
<TranslationToggle 
  docId={metadata.id} 
  version="1.0"
  onTranslate={(urduContent) => {
    // Handle the translated content
    document.querySelector('.markdown').innerHTML = urduContent;
  }}
/>
```

## Step 3: Implement the Translation API

Create the backend endpoint that handles translation requests.

**For FastAPI Backend**:
- Copy `assets/translate_api.py` to your backend directory
- Configure OpenAI API key: `export OPENAI_API_KEY=your_key`
- Set up database schema for caching (see Step 4)

**For Next.js API Routes**:
- Create `pages/api/translate.ts` with similar logic
- Use Vercel's serverless functions

**Key Requirements**:
- Accept: `doc_id`, `version`, `lang`, `content`
- Check cache before calling OpenAI
- Return: `translated`, `cached`, `source`, `created_at`
- Store result in cache for future requests

## Step 4: Set Up Translation Caching

Choose a caching strategy to avoid repeated translation costs:

**Option A: Database Cache (Recommended for Dynamic Sites)**

Create this schema in your Neon Postgres database:

```sql
CREATE TABLE translation_cache (
  doc_id TEXT NOT NULL,
  version TEXT NOT NULL,
  lang TEXT NOT NULL,
  content TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  PRIMARY KEY (doc_id, version, lang)
);

CREATE INDEX idx_translation_lookup ON translation_cache(doc_id, version, lang);
```

**Option B: File Cache (For Static Sites)**

Generate translations at build time:
```
/translations/
  ur/
    module-1/
      chapter-1.md
      chapter-2.md
```

**Option C: Browser Storage (Supplementary)**

Add client-side caching in the component:
```typescript
localStorage.setItem(`translation_${docId}_${version}_ur`, translatedContent);
```

## Step 5: Add RTL Styling Support

Ensure Urdu text renders correctly with right-to-left direction.

Create `src/css/custom.css` additions:

```css
/* RTL support for Urdu content */
[dir="rtl"] {
  direction: rtl;
  text-align: right;
}

[dir="rtl"] .markdown {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', Arial, sans-serif;
}

/* Preserve code blocks in LTR */
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}

/* Adjust navigation for RTL */
[dir="rtl"] .sidebar {
  right: 0;
  left: auto;
}
```

**Font Loading**: Add to your `docusaurus.config.js`:

```javascript
stylesheets: [
  {
    href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu&display=swap',
    rel: 'stylesheet'
  }
]
```

## Step 6: Handle Translation State

Manage which language is currently displayed:

**In Component State**:
```typescript
const [isUrdu, setIsUrdu] = useState(false);
const [originalContent, setOriginalContent] = useState('');

const toggleLanguage = () => {
  if (isUrdu) {
    // Restore English
    contentElement.innerHTML = originalContent;
    contentElement.removeAttribute('dir');
  } else {
    // Save English and show Urdu
    setOriginalContent(contentElement.innerHTML);
    contentElement.innerHTML = translatedContent;
    contentElement.setAttribute('dir', 'rtl');
  }
  setIsUrdu(!isUrdu);
};
```

## Step 7: Add Translation Quality Label

Always label machine-translated content for transparency:

```tsx
{isUrdu && (
  <div className="translation-notice" style={{
    background: '#fff3cd',
    border: '1px solid #ffc107',
    borderRadius: '4px',
    padding: '8px 12px',
    marginBottom: '16px',
    fontSize: '0.9em'
  }}>
    🤖 This content was machine-translated from English using AI. 
    Some technical terms may vary.
  </div>
)}
```

## Step 8: Test and Validate

Run the validation script to ensure everything is configured correctly:

```bash
python scripts/validate_translation.py /path/to/your/project
```

**Manual Testing Checklist**:
- [ ] Button appears on all chapter pages
- [ ] Click button translates content to Urdu
- [ ] Urdu text renders right-to-left
- [ ] Click again returns to English
- [ ] Translation quality label is visible
- [ ] Code blocks remain left-to-right
- [ ] Navigation still works when in Urdu mode
- [ ] Cached translations load instantly on second click

## Advanced: User Personalization

If users are logged in (Better Auth integration), you can:

1. **Remember Language Preference**: Store user's language choice in their profile
2. **Auto-translate on Page Load**: If user prefers Urdu, translate automatically
3. **Track Translation Usage**: Understand which chapters users translate most

```typescript
// On translation toggle:
await fetch('/api/user/preferences', {
  method: 'PATCH',
  body: JSON.stringify({ language: isUrdu ? 'ur' : 'en' })
});
```

## Non-Negotiable Rules

1. **User Control**: User must explicitly request translation (button click)
2. **Preserve Original**: Always keep English content accessible
3. **Label Machine Translation**: Clearly indicate AI-generated content
4. **No Layout Breaking**: RTL must not break navigation, sidebar, or responsive design
5. **Cache Translations**: Never translate the same content twice unnecessarily
6. **Code Preservation**: Keep code blocks and technical syntax in English

## Troubleshooting

**Issue**: Urdu text appears broken or boxes
- **Fix**: Ensure Urdu font is loaded and applied with `font-family`

**Issue**: Translation button doesn't appear
- **Fix**: Check component is imported in chapter template, verify routing

**Issue**: API returns 401/403
- **Fix**: Verify OpenAI API key is set and valid

**Issue**: Layout breaks in RTL mode
- **Fix**: Audit CSS for hardcoded `left`/`right` values, use `inline-start`/`inline-end`

**Issue**: Sidebar disappears in Urdu
- **Fix**: Add RTL-specific CSS for `.sidebar` positioning

## Resources

This skill includes:

- **scripts/validate_translation.py**: Automated validation of translation implementation
- **references/translation_patterns.md**: Detailed UX patterns, caching strategies, and technical considerations
- **assets/TranslationToggle.tsx**: Example React component for translation button
- **assets/translate_api.py**: Example FastAPI endpoint for translation service

## Cost Considerations

OpenAI API costs for translation:
- GPT-4: ~$0.03 per 1K tokens input, $0.06 per 1K tokens output
- Average chapter (2000 words): ~$0.15-0.30 per translation
- **Mitigation**: Aggressive caching is essential
- **Alternative**: Consider GPT-3.5-turbo for 10x lower cost with acceptable quality

## Acceptance Checklist

✅ Translation button exists on every chapter page
✅ Clicking button translates content to Urdu via API
✅ Urdu renders correctly with RTL direction and proper fonts
✅ User can switch back to English with same button
✅ Translations are cached (database or file system)
✅ Machine translation label is clearly visible
✅ Code blocks and technical terms preserve formatting
✅ Navigation and sidebar work correctly in both languages
✅ Mobile responsive in both English and Urdu modes