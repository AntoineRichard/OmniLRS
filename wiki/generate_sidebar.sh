#!/bin/bash

# Usage: ./generate_sidebar.sh input_markdown_file.md

# Check if the input file is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <input_markdown_file.md>"
    exit 1
fi

# Input markdown file
input_file="$1"
output_folder="$2"

# Output sidebar file
output_file=$2/"_Sidebar.md"

# Start with an empty _Sidebar.md file
> "$output_file"

echo "## Table of Contents" >> "$output_file"
echo "" >> "$output_file"

# Read the markdown file line by line
while IFS= read -r line; do

    # Check for headers (lines starting with #)
    if [[ "$line" =~ ^(#{1,6})\ (.*) ]]; then
        # Count the number of # to determine the heading level
        level="${#BASH_REMATCH[1]}"
        title="${BASH_REMATCH[2]}"
        
        # Convert title to lowercase, replace spaces with hyphens for anchor links
        link=$(echo "$title" | tr '[:upper:]' '[:lower:]' | tr -d '[:punct:]' | tr ' ' '-')

        # Create indentation based on heading level
        indentation=$(printf '%*s' $(($((level - 1)) * 2)) '')

        # Add the header link to the _Sidebar.md file
        echo "${indentation}- [${title}](#${link})" >> "$output_file"
    fi

done < "$input_file"

echo "_Sidebar.md has been generated from $input_file."

