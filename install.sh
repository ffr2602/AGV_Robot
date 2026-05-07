#!/bin/bash

set -e

SERVICE_NAME="agv_robot"
INSTALL_DIR="/opt/agv_robot"

# Folder source (hasil git clone)
SOURCE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "================================="
echo "AGV Robot Installer"
echo "================================="

echo "Source : $SOURCE_DIR"
echo "Target : $INSTALL_DIR"

# Install rsync jika belum ada
if ! command -v rsync &> /dev/null
then
    echo "Installing rsync..."
    apt update
    apt install -y rsync
fi

# Stop service lama jika ada
systemctl stop $SERVICE_NAME 2>/dev/null || true

# Buat folder target
mkdir -p $INSTALL_DIR

echo "Sync files..."

# Sync project
rsync -a --delete \
    --exclude '.git' \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    $SOURCE_DIR/ $INSTALL_DIR/

# Permission
chmod +x $INSTALL_DIR/run.py

echo "Create systemd service..."

cat <<EOF > /etc/systemd/system/${SERVICE_NAME}.service
[Unit]
Description=AGV Robot Service
After=network.target

[Service]
Type=simple
WorkingDirectory=${INSTALL_DIR}
ExecStart=/usr/bin/python3 ${INSTALL_DIR}/run.py

Restart=always
RestartSec=3

User=root

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd
systemctl daemon-reload

# Enable auto boot
systemctl enable $SERVICE_NAME

# Restart service
systemctl restart $SERVICE_NAME

echo ""
echo "================================="
echo "INSTALL SUCCESS"
echo "================================="

systemctl status $SERVICE_NAME --no-pager