#!/bin/sh
# Turn on this project's git hooks (macOS, Linux, or Git Bash on Windows).
# Run once, from the project directory:   ./setup-hooks.sh
git rev-parse --git-dir >/dev/null 2>&1 || {
    echo "This is not a git repository, so there is nothing to set up."
    exit 1
}
git config core.hooksPath .githooks
echo "Hooks enabled. confirmation.txt will be checked before each commit."
