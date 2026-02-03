# Spec-Driven Development with Numbered Branches

## Overview
The `sp.specify` command now automatically creates branches based on the next available spec number in the sequence. This ensures orderly progression and prevents conflicts between different feature developments.

## How it Works

1. **Automatic Numbering**: The system scans existing spec directories (`specs/`) and git branches to find the highest number in use
2. **Next Available Number**: It calculates the next number (highest + 1) and formats it as a 3-digit number (e.g., 007)
3. **Branch Creation**: Creates a branch with the format `{NNN}-{short-name}` (e.g., `007-add-user-authentication`)
4. **Directory Structure**: Creates the corresponding spec directory and files

## Example Usage

```bash
# From any branch, run:
sp.specify "Add user authentication with OAuth2"

# The system will:
# 1. Detect that 006 is the highest existing number
# 2. Generate the next number: 007
# 3. Create branch: 007-add-user-authentication
# 4. Create directory: specs/007-add-user-authentication/
# 5. Initialize: specs/007-add-user-authentication/spec.md
```

## Benefits

- **Orderly Progression**: Ensures specs are created in sequence
- **Conflict Prevention**: Prevents multiple people from using the same number
- **Easy Tracking**: Clear relationship between branch number and spec number
- **Consistent Naming**: Standardized format across the project