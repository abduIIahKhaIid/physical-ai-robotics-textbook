#!/usr/bin/env bash

# Script to list all specs in numerical order with their status

echo "Current Specs Structure:"
echo "========================"

# Find all spec directories and sort them numerically
for spec_dir in $(find specs/ -maxdepth 1 -type d -name '[0-9]*' | sort -V); do
    spec_num=$(basename "$spec_dir" | cut -d'-' -f1)
    spec_name=$(basename "$spec_dir" | cut -d'-' -f2-)

    # Check if spec.md exists in the directory
    if [ -f "$spec_dir/spec.md" ]; then
        status="✓"
    else
        status="○"
    fi

    # Check if there's a branch with this number
    current_branch=$(git branch --show-current 2>/dev/null || echo "")
    if [[ "$current_branch" =~ ^$spec_num-[a-zA-Z] ]]; then
        branch_status="(*)"
    else
        branch_status="   "
    fi

    printf "%-3s %-4s %-40s %s\n" "$branch_status" "$spec_num" "$spec_name" "$status"
done

echo ""
echo "Legend:"
echo "  (*) = Current branch"
echo "  ✓ = spec.md exists"
echo "  ○ = Directory exists, spec.md missing"