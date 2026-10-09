#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPLICATIONS_DIR="${XDG_DATA_HOME:-${HOME}/.local/share}/applications"
mkdir -p "${APPLICATIONS_DIR}"
python3 - "${SCRIPT_DIR}/ai_pc_open_rviz.py" "${APPLICATIONS_DIR}/h1-rviz.desktop" <<'PY'
from pathlib import Path
import sys
launcher, target = sys.argv[1:]
# Desktop Exec quoting has its own escaping rules, separate from shell quoting.
quoted = launcher.replace('\\', '\\\\\\\\').replace('"', '\\\\"').replace('`', '\\\\`').replace('$', '\\\\$').replace('%', '%%')
Path(target).write_text(
    '[Desktop Entry]\nType=Application\nName=H1 Local RViz\n'
    f'Exec=/usr/bin/python3 "{quoted}" %u\n'
    'Terminal=false\nNoDisplay=true\nMimeType=x-scheme-handler/h1-rviz;\n',
    encoding='utf-8',
)
PY
if command -v update-desktop-database >/dev/null; then
    update-desktop-database "${APPLICATIONS_DIR}"
fi
xdg-mime default h1-rviz.desktop x-scheme-handler/h1-rviz
xdg-mime query default x-scheme-handler/h1-rviz
