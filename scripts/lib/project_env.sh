# Shared JSON settings, quoted by Python; configuration is never sourced as code.
_jetpilot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
_jetpilot_exports="$(python3 "${_jetpilot_root}/scripts/lib/project_config.py" --root "${_jetpilot_root}")" || return
eval "${_jetpilot_exports}"
unset _jetpilot_root _jetpilot_exports
