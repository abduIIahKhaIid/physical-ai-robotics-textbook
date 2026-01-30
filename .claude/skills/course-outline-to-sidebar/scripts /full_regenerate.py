#!/usr/bin/env python3
"""
Full Regeneration Workflow

Runs the complete pipeline from course outline to validated structure.

Usage:
    full_regenerate.py <outline.json> <docs_dir> [--validate]

Example:
    full_regenerate.py outline.json docs/ --validate
"""

import sys
import subprocess
from pathlib import Path


def run_command(cmd: list, description: str) -> bool:
    """Run a command and report status."""
    print(f"\n{'='*60}")
    print(f"🚀 {description}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=False)
        print(f"✅ {description} - SUCCESS")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} - FAILED")
        print(f"   Error: {e}")
        return False


def main():
    if len(sys.argv) < 3:
        print("Usage: full_regenerate.py <outline.json> <docs_dir> [--validate]")
        sys.exit(1)
    
    outline_file = Path(sys.argv[1])
    docs_dir = Path(sys.argv[2])
    validate = '--validate' in sys.argv
    
    if not outline_file.exists():
        print(f"❌ Error: outline file does not exist: {outline_file}")
        sys.exit(1)
    
    script_dir = Path(__file__).parent
    
    print("🎯 Starting Full Regeneration Pipeline")
    print(f"   Outline: {outline_file}")
    print(f"   Docs:    {docs_dir}")
    print(f"   Validate: {validate}")
    
    # Step 1: Generate directory structure
    if not run_command(
        ['python3', str(script_dir / 'generate_structure.py'), str(outline_file), str(docs_dir)],
        'Generating directory structure'
    ):
        sys.exit(1)
    
    # Step 2: Generate MDX stubs
    if not run_command(
        ['python3', str(script_dir / 'generate_stubs.py'), str(outline_file), str(docs_dir)],
        'Generating MDX stub files'
    ):
        sys.exit(1)
    
    # Step 3: Generate sidebar
    sidebars_file = docs_dir.parent / 'sidebars.ts'
    if not run_command(
        ['python3', str(script_dir / 'generate_sidebar.py'), str(outline_file), '--output', str(sidebars_file)],
        'Generating sidebars.ts'
    ):
        sys.exit(1)
    
    # Step 4: Validate (if requested)
    if validate:
        if not run_command(
            ['python3', str(script_dir / 'validate_structure.py'), str(docs_dir), str(sidebars_file)],
            'Validating structure'
        ):
            print("\n⚠️  Validation found issues, but generation was successful")
            print("   Review the warnings and errors above")
    
    print("\n" + "="*60)
    print("✅ Full regeneration complete!")
    print("="*60)
    print(f"\nNext steps:")
    print(f"  1. Review generated files in: {docs_dir}")
    print(f"  2. Review sidebar config: {sidebars_file}")
    print(f"  3. Start writing content in the MDX files")
    print(f"  4. Run your Docusaurus dev server to preview")


if __name__ == '__main__':
    main()