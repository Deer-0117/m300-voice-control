#!/bin/bash
# Install only logging retention. Does not restart aircraft/control services.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then echo 'Run with sudo.' >&2; exit 1; fi
mkdir -p /etc/systemd/journald.conf.d /var/log/journal
destination=/etc/systemd/journald.conf.d/60-balloon-flight.conf
if [ -f "$destination" ]; then cp -p "$destination" "$destination.backup.$(date +%s)"; fi
cat > "$destination" <<'EOF'
[Journal]
Storage=persistent
SystemMaxUse=512M
SystemKeepFree=1G
MaxRetentionSec=30day
SyncIntervalSec=5s
EOF
systemd-tmpfiles --create --prefix /var/log/journal
if id uav >/dev/null 2>&1; then usermod -a -G systemd-journal uav; fi
systemctl restart systemd-journald
journalctl --flush
journalctl --sync
echo 'Persistent journal configured (512 MiB / 30 days, subject to disk space).'
echo 'After a reboot, verify journalctl --list-boots includes the preceding boot.'
echo 'Reconnect SSH for the uav journal-read group membership to take effect.'
journalctl --disk-usage
