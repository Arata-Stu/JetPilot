#!/usr/bin/env bash
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/80-movidius.rules"

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

"${SUDO[@]}" tee "${RULE_FILE}" >/dev/null <<'EOF'
# Luxonis OAK / Intel Movidius devices
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE:="0666", GROUP="plugdev", TAG+="uaccess"
EOF

"${SUDO[@]}" udevadm control --reload-rules
"${SUDO[@]}" udevadm trigger

echo "Successfully added Luxonis OAK udev rules: ${RULE_FILE}"
echo "Unplug and replug the OAK device, then restart the Docker container."
