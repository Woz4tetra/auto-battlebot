#!/bin/bash


install_uvcvideo_rt() {
    local service_name="uvcvideo-rt.service"
    local service_file="/etc/systemd/system/${service_name}"
    local keeper="/usr/local/sbin/auto-battlebot-uvcvideo-rt"

    echo "Installing ${keeper} and ${service_name}..."
    sudo tee "$keeper" >/dev/null <<'EOF'
#!/bin/bash
# Holds uvcvideo's copy kworkers at SCHED_FIFO 50. See install/install_uvcvideo_rt.sh.
while true; do
    ps -eLo pid=,cls=,comm= |
        awk '$2 == "TS" && $3 ~ /^kworker\/.*uvcvideo$/ { print $1 }' |
        while read -r pid; do chrt -f -p 50 "$pid" 2>/dev/null || true; done
    sleep 1
done
EOF
    sudo chmod 755 "$keeper"

    sudo tee "$service_file" >/dev/null <<EOF
[Unit]
Description=Keep uvcvideo kworkers at SCHED_FIFO (ZED frame rotation fix)

[Service]
Type=simple
ExecStart=${keeper}
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable "$service_name"
    sudo systemctl restart "$service_name"

    echo ""
    echo "${service_name} setup complete."
    echo "Verification, with the camera streaming:"
    echo "  - ps -eLo pid,cls,rtprio,comm | grep uvcvideo   # every row FF 50"
}
