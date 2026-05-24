#! /bin/bash

###############################################################################
# Install and configure Tailscale on the RDK X3 (Ubuntu 20.04 arm64).
#
# Tailscale is a WireGuard-based mesh VPN that gives every device a stable
# 100.x.x.x IP reachable from any network, with no port forwarding or DDNS.
# We use it so the VR (Quest 3) and the mobile app can talk to the robot
# over the Internet.
#
# Usage (on the robot):
#   sudo bash install_tailscale.sh
#
# First run is interactive: the script prints a login URL that must be
# opened in a browser and authenticated against your Tailscale account.
# After that the daemon auto-starts on boot and reconnects automatically.
###############################################################################

set -e

if [ "$EUID" -ne 0 ]; then
  echo "This script must be run as root (use sudo)."
  exit 1
fi

# 1) install tailscale (idempotent: re-running is fine)
if ! command -v tailscale >/dev/null 2>&1; then
  echo "Installing Tailscale..."
  curl -fsSL https://tailscale.com/install.sh | sh
else
  echo "Tailscale already installed: $(tailscale version | head -n1)"
fi

# 2) make sure the daemon is enabled and running
systemctl enable --now tailscaled

# 3) bring the node up.
#  --hostname:      stable name so MagicDNS gives us e.g. robot.<tail>.ts.net
#  --ssh:           enable Tailscale SSH (auth via tailnet, no extra keys).
#                   Remove this flag if you prefer plain sshd only.
#  --accept-dns=false: keep the robot's existing DNS (it hosts its own
#                   hotspot DHCP/DNS, see hotspot/etc/dhcp/dhcpd.conf,
#                   we don't want Tailscale overriding that).
tailscale up \
  --hostname=robot \
  --ssh \
  --accept-dns=false

# 4) print the resulting connection info
echo ""
echo "============================================================"
echo "Tailscale is up. Connection info:"
echo "  Tailscale IPv4: $(tailscale ip -4)"
echo "  MagicDNS name:  $(tailscale status --self --json 2>/dev/null | grep -oP '\"DNSName\":\s*\"\K[^\"]+' | head -n1)"
echo ""
echo "Clients (Unity/VR, mobile app) should connect to one of the above"
echo "instead of the LAN/hotspot IP. Ports unchanged:"
echo "  ROS-TCP-Endpoint (Unity/VR):   10000"
echo "  (mobile app uses its own existing port)"
echo "============================================================"
