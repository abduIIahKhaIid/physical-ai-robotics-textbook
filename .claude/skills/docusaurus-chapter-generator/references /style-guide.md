# Writing Style Guide

This reference covers writing conventions for educational content.

## Voice and Tone

### Use Second Person ("You")
✅ **Good**: "You will learn how to create functions in Python."
❌ **Avoid**: "We will learn..." or "The student will learn..."

### Use Present Tense
✅ **Good**: "Python uses indentation to define code blocks."
❌ **Avoid**: "Python will use indentation..." or "Python used to use..."

### Use Active Voice
✅ **Good**: "The function returns a value."
❌ **Avoid**: "A value is returned by the function."

### Be Direct and Conversational
✅ **Good**: "Let's explore how loops work."
❌ **Avoid**: "It is now necessary to examine the mechanics of iterative constructs."

## Technical Writing Standards

### Code Terminology
- Use `backticks` for inline code: "Use the `print()` function"
- Use code blocks for multi-line examples
- Capitalize proper nouns: Python, JavaScript, Django
- Don't capitalize: string, boolean, integer (unless at sentence start)

### Formatting Code Examples
```markdown
```python title="example.py"
def greet(name):
    """Display a greeting message."""
    return f"Hello, {name}!"

# Call the function
message = greet("Alice")
print(message)  # Output: Hello, Alice!
```
\```
```

### Explaining Code
- **Before the code**: Explain what you're about to show
- **In comments**: Clarify non-obvious logic
- **After the code**: Explain the result or significance

Example structure:
```markdown
To reverse a string, you can use Python's slicing syntax:

```python
text = "Hello"
reversed_text = text[::-1]  # Slice with step of -1
print(reversed_text)  # Output: olleH
```

This works because negative step values traverse the string backwards.
\```
```

## Common Terms and Spelling

### Consistent Terminology
- **Function** (not method, unless specifically OOP context)
- **Argument** vs **Parameter**: Parameters are in definitions, arguments are passed
- **Variable** (not var)
- **String** (not str, unless referring to the type name)
- **List** (in Python) vs **Array** (in JavaScript)

### Spelling and Capitalization
- Email (not e-mail)
- Website (not web site)
- Username (not user name)
- Backend, frontend (not back-end, front-end as adjectives)
- API (all caps)
- URL (all caps)
- JSON, HTML, CSS, SQL (all caps)

## Numbers and Symbols

### Numbers
- Spell out one through nine: "three variables"
- Use numerals for 10 and above: "15 functions"
- Always use numerals with units: "5 MB", "3 seconds"
- Use commas in numbers ≥ 1,000: "1,000 users"

### Symbols
- Use × for multiplication in prose: "5 × 3"
- Use * in code examples: `5 * 3`
- Use % for percentages: "50% complete"

## Lists and Sequences

### Bulleted Lists
Use for unordered items:
- Item one
- Item two
- Item three

### Numbered Lists
Use for sequences, steps, or ranked items:
1. First step
2. Second step
3. Third step

### List Punctuation
- No period after short phrases
- Include periods for complete sentences
- Use parallel structure (all items same grammatical form)

## Headers and Structure

### Header Hierarchy
```markdown
# Chapter Title (h1) - Only one per page

## Main Section (h2) - Major topics

### Subsection (h3) - Supporting details

#### Sub-subsection (h4) - Rarely needed
```

### Header Capitalization
Use sentence case:
- ✅ "Understanding variables and data types"
- ❌ "Understanding Variables and Data Types"

Exception: Proper nouns are always capitalized
- ✅ "Introduction to Python syntax"

## Links and References

### Internal Links
```markdown
For more information, see [Variables and Data Types](./variables.md).
```

### External Links
```markdown
Learn more in the [official Python documentation](https://docs.python.org).
```

### Link Text
- ✅ "Read the [installation guide](./install.md)"
- ❌ "Click [here](./install.md) for the installation guide"

## Admonitions Usage

### When to Use Each Type

**:::tip** - Best practices, pro tips, efficiency tricks
```markdown
:::tip
Use descriptive variable names like `user_count` instead of `uc`.
:::
```

**:::warning** - Common mistakes, gotchas, important caveats
```markdown
:::warning
Don't modify a list while iterating over it - this causes unexpected behavior.
:::
```

**:::info** - Supplementary information, context, "did you know?"
```markdown
:::info
Python was created by Guido van Rossum and first released in 1991.
:::
```

**:::danger** - Critical errors, security issues, data loss risks
```markdown
:::danger
Never store passwords in plain text. Always use proper hashing.
:::
```

## Inclusive Language

### Use Gender-Neutral Examples
- ✅ "The user clicks their profile"
- ✅ "A developer writes their code"
- ❌ "The user clicks his profile"

### Vary Example Names
Use diverse names in examples:
- Alice, Bob, Charlie, Diana, Elena, Frank
- Aisha, Carlos, Priya, Jin, Fatima

### Avoid Assumptions
- ✅ "Users might prefer..."
- ❌ "Users will obviously want..."

## Error Messages and Troubleshooting

### Format Error Messages
```markdown
If you see this error:
```
ERROR: ModuleNotFoundError: No module named 'requests'
```

It means Python can't find the requests library. Install it:
```bash
pip install requests
```
\```
```

### Troubleshooting Structure
1. **Symptom**: What the user sees
2. **Cause**: Why it's happening
3. **Solution**: How to fix it
4. **Prevention**: How to avoid it in the future

## Examples and Analogies

### Good Analogies
- Relatable to target audience
- Accurate to the concept
- Don't oversimplify to the point of incorrectness

Example:
```markdown
Think of a function as a recipe. It takes ingredients (arguments), 
follows a series of steps (the function body), and produces a dish 
(the return value).
```

### Bad Analogies
- Overly complex metaphors
- Confusing multiple concepts
- Culturally specific references

## Final Formatting Checklist

- [ ] Headers follow hierarchy (h1 → h2 → h3)
- [ ] Code blocks specify language
- [ ] Inline code uses backticks
- [ ] Lists use parallel structure
- [ ] Links have descriptive text
- [ ] Tone is conversational but professional
- [ ] Technical terms are used consistently
- [ ] Examples use varied, inclusive names
- [ ] No passive voice in instructions
- [ ] Second person ("you") used throughout