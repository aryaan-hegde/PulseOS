@echo off
REM Turn on this project's git hooks (Windows, cmd or PowerShell).
REM Run once, from the project folder:   setup-hooks.bat
git rev-parse --git-dir >nul 2>&1
if errorlevel 1 (
    echo This is not a git repository, so there is nothing to set up.
    exit /b 1
)
git config core.hooksPath .githooks
echo Hooks enabled. confirmation.txt will be checked before each commit.
