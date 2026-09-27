#!/usr/bin/env bash
# 部署 NWCT (lanproxy) 内网穿透客户端
# 用法: sudo bash /home/kickpi/rl_sar-w2_recovered/deploy_nwct.sh
set -eo pipefail

KEY="ed034817e23f42589900bfd24e1754f4"
SRC_BIN="/home/kickpi/.trae-cn-server/data/User/workspaceStorage/e9ae7be1054dfa001179a6a85107a67d/external-files/6aa6591f916aadd6356bd9bc/2-mtzks83s-wtro/main_linux_arm_V.20250626204827"
APP_DIR="/opt/nwct"
BIN_NAME="main_linux_arm_V.20250626204827"

mkdir -p "$APP_DIR"
install -m 0755 "$SRC_BIN" "$APP_DIR/$BIN_NAME"

# 手动启动脚本（保留，兼容原 nwct 的 /opt/nwct/start.sh 方式）
cat > "$APP_DIR/start.sh" <<EOF
#!/bin/bash
cd "\$(dirname "\$0")"
exec ./$BIN_NAME -k $KEY
EOF
chmod +x "$APP_DIR/start.sh"

# systemd 服务单元（本机为 systemd，用 service 比 SysV init 更可靠）
cat > /etc/systemd/system/nwct.service <<EOF
[Unit]
Description=NWCT lanproxy client (内网穿透)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=$APP_DIR/$BIN_NAME -k $KEY
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now nwct.service

echo "=== 部署完成，服务状态 ==="
systemctl status nwct.service --no-pager || true
