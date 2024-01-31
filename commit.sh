# Check if a commit message is provided
if [ -z "$1" ]; then
    echo "Please provide a commit message."
    exit 1
fi

pre-commit run

# Check if pre-commit modified any files
modified_files=$(git diff --name-only --cached)

# If pre-commit modified files, restage them
if [ -n "$modified_files" ]; then
    git add $modified_files
fi

pre-commit run

commit_message="$1"
git commit -m "$commit_message"
