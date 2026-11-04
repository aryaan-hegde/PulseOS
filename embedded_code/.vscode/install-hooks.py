# Turn on this project's git hooks, once, at build time.
#
# Git will not let a repository install its own hooks - cloning would otherwise
# run its code - so somebody has to point git at .githooks. Doing it here means
# it happens the first time you build and you never have to think about it.
#
# The hooks themselves stay quiet: the pre-commit hook only speaks up if
# confirmation.txt looks wrong (see .githooks/pre-commit).
#
# Nothing in here can stop your project building. Every failure is swallowed:
# no git, no repository, a read-only checkout, or being unpacked somewhere
# without .githooks at all - in each case this does nothing and says nothing.

import json
import os
import re
import subprocess

try:
    Import("env")                                   # noqa: F821
    root = env.subst("$PROJECT_DIR")                # noqa: F821
except Exception:
    root = os.getcwd()

try:
    if (os.path.isdir(os.path.join(root, ".githooks")) and
            os.path.isdir(os.path.join(root, ".git"))):
        current = subprocess.run(
            ["git", "config", "--get", "core.hooksPath"],
            cwd=root, capture_output=True, text=True, timeout=10)
        if current.stdout.strip() != ".githooks":
            subprocess.run(["git", "config", "core.hooksPath", ".githooks"],
                           cwd=root, capture_output=True, timeout=10)
            print("Enabled this project's git hooks; "
                  "confirmation.txt will be checked before each commit.")
except Exception:
    pass                                            # never break the build


DESIRED_VSCODE_SETTINGS = {
    "chat.disableAIFeatures": True,
    "chat.agent.enabled": False,
    "chat.commandCenter.enabled": False,
    "github.copilot.enable": {"*": False},
    "github.copilot.editor.enableAutoCompletions": False,
    "github.copilot.inlineSuggest.enable": False,
    "extensions.ignoreRecommendations": True,
}

try:
    settings_path = os.path.join(root, ".vscode", "settings.json")
    current = {}
    if os.path.isfile(settings_path):
        with open(settings_path) as f:
            text = f.read().strip()
        if text:
            current = json.loads(text)

    merged = dict(current)
    merged.update(DESIRED_VSCODE_SETTINGS)

    if merged != current:
        os.makedirs(os.path.dirname(settings_path), exist_ok=True)
        with open(settings_path, "w") as f:
            json.dump(merged, f, indent=4)
            f.write("\n")
except Exception:
    pass                                            # never break the build


DESIRED_UNWANTED_EXTENSIONS = [
    "anthropic.claude-code",
    "GitHub.copilot",
    "GitHub.copilot-chat",
    "saoudrizwan.claude-dev",
    "RooVeterinaryInc.roo-cline",
    "Continue.continue",
    "Codeium.CodeiumVS",
    "sourcegraph.cody-ai",
    "TabNine.tabnine-vscode",
    "AmazonWebServices.amazon-q-vscode",
    "Google.geminicodeassist",
    "supermaven.supermaven",
    "Codium.codium",
    "augment.vscode-augment",
]

try:
    extensions_path = os.path.join(root, ".vscode", "extensions.json")
    current = {}
    if os.path.isfile(extensions_path):
        with open(extensions_path) as f:
            text = f.read()
        stripped = re.sub(r"^\s*//.*$", "", text, flags=re.M).strip()
        if stripped:
            current = json.loads(stripped)

    unwanted = list(current.get("unwantedRecommendations", []))
    for ext_id in DESIRED_UNWANTED_EXTENSIONS:
        if ext_id not in unwanted:
            unwanted.append(ext_id)

    merged = dict(current)
    merged["unwantedRecommendations"] = unwanted

    if merged != current:
        os.makedirs(os.path.dirname(extensions_path), exist_ok=True)
        with open(extensions_path, "w") as f:
            json.dump(merged, f, indent=4)
            f.write("\n")
except Exception:
    pass                                            # never break the build


#
# Check confirmation.txt here as well as in the pre-commit hook.
#
# Graphical git clients bury hook output - VS Code hides it behind a "Show
# Command Output" button - so a student using the Source Control panel may only
# see that the commit failed. Building is the one thing everybody does and
# watches the output of, so say it here too. This never fails the build: it is
# a warning, and the hook is what actually blocks a bad commit.
#
try:
    conf = os.path.join(root, "confirmation.txt")
    if os.path.isfile(conf):
        with open(conf) as f:
            code = "".join(f.read().split())
        if code:
            problem = None
            if "onfirmation" in code or "code:" in code:
                problem = ("it still has the words from the prompt in it - "
                           "paste only the hex code")
            elif any(c not in "0123456789abcdefABCDEF" for c in code):
                problem = "it contains characters that are not hex digits"
            elif len(code) != 256:
                problem = ("it is %d characters long but should be 256" %
                           len(code))
            if problem:
                print("")
                print("  WARNING: confirmation.txt does not look like a "
                      "complete confirmation code:")
                print("  %s." % problem)
                print("  Copy the whole line after 'Confirmation code:' into "
                      "it, with no spaces")
                print("  or line breaks, or the autograder will reject your "
                      "submission.")
                print("")
except Exception:
    pass


#
# Check that main.c's `username` variable has been changed from the
# placeholder before building. Same reasoning as the confirmation.txt check
# above: this never fails the build, it just makes the reminder impossible
# to miss.
#
try:
    main_c = os.path.join(root, "src", "main.c")
    if os.path.isfile(main_c):
        with open(main_c) as f:
            src = f.read()
        m = re.search(r'const\s+char\s*\*\s*username\s*=\s*"([^"]*)"\s*;', src)
        if m:
            value = m.group(1).strip()
            problem = None
            if value == "" or value == "username":
                problem = "it is still set to the placeholder \"username\""
            elif "@" in value or any(c.isspace() for c in value):
                problem = ("it should be just your Purdue career account "
                           "alias, e.g. \"msmith42\" - not an email address "
                           "or a name with spaces")
            if problem:
                print("")
                print("  WARNING: the `username` variable in src/main.c "
                      "does not look right:")
                print("  %s." % problem)
                print("  Change it to your own Purdue username, e.g.:")
                print("      const char* username = \"msmith42\";")
                print("")
except Exception:
    pass                                            # never break the build
