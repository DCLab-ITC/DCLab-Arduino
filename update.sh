#!/usr/bin/env bash
# Updates this library to the latest version on GitHub (Linux, macOS).
# Run it from the library's folder (Arduino/libraries/DCLab):
#
#   ./update.sh            from GitHub
#   ./update.sh FILE.zip   from a DCLab-Arduino.zip you already have
#
# A git clone is pulled (it stops if you changed files in it); otherwise the
# folder is replaced with the new version. On Windows run update.cmd.
set -euo pipefail

REPO=https://github.com/DCLab-ITC/DCLab-Arduino
here=$(cd "$(dirname "$0")" && pwd)
version() { sed -n 's/^version=//p' "$1/library.properties" 2>/dev/null; }
before=$(version "$here")

if [ $# -eq 0 ] && [ -d "$here/.git" ]; then
  if [ -n "$(git -C "$here" status --porcelain --untracked-files=no)" ]; then
    echo "You changed files in $here (git status): commit or undo them, then run this again." >&2
    exit 1
  fi
  git -C "$here" pull --ff-only "$REPO" main
else
  work=$(mktemp -d)
  trap 'rm -rf "$work"' EXIT
  if [ $# -gt 0 ]; then
    cp "$1" "$work/new.zip"
  else
    echo "Downloading $REPO ..."
    if command -v curl >/dev/null; then
      curl -fsSL -o "$work/new.zip" "$REPO/archive/refs/heads/main.zip"
    else
      wget -q -O "$work/new.zip" "$REPO/archive/refs/heads/main.zip"
    fi
  fi
  if command -v unzip >/dev/null; then
    unzip -q "$work/new.zip" -d "$work/new"
  else
    python3 -c 'import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])' \
      "$work/new.zip" "$work/new"
  fi
  # one top folder: DCLab/ (Super App) or DCLab-Arduino-main/ (GitHub)
  new=$(find "$work/new" -mindepth 1 -maxdepth 1 -type d | head -n 1)
  if ! grep -qx "name=DCLab" "$new/library.properties" 2>/dev/null; then
    echo "That is not the DCLab library." >&2
    exit 1
  fi
  chmod +x "$new/update.sh" 2>/dev/null || true
  # the old copy goes, not next to it: the IDE would find the library twice
  mv "$here" "$work/old"
  mv "$new" "$here"
fi

echo "DCLab library ${before:-?} -> $(version "$here")"
echo "Done. Restart the Arduino IDE if it is open."
