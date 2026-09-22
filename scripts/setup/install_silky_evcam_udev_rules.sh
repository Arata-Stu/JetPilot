#!/usr/bin/env bash
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-silky-evcam.rules"

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

"${SUDO[@]}" tee "${RULE_FILE}" >/dev/null <<'EOF'
# CenturyArks SilkyEvCam Gen3.1
SUBSYSTEM=="usb", ATTR{idVendor}=="31f7", ATTR{idProduct}=="0002", MODE:="0666", GROUP="plugdev", TAG+="uaccess"
EOF

"${SUDO[@]}" udevadm control --reload-rules
"${SUDO[@]}" udevadm trigger

echo "Successfully added SilkyEvCam udev rules: ${RULE_FILE}"
echo "Unplug and replug the SilkyEvCam device, then restart the Docker container."
