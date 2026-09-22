#!/bin/bash

# Hold uvcvideo's copy kworkers at SCHED_FIFO so the ZED's bulk endpoint never overruns.
#
# Bug: on JetPack 7 (kernel 6.8) ~0.7% of ZED 2i frames arrive with the whole byte stream
# shifted by 2048 bytes (two 1024 B bulk packets). The image looks cyclically rotated with a
# seam at column 1024. The shift is already in the V4L2 buffer at DQBUF, before the ZED SDK
# touches it.
#
# Cause: uvcvideo copies each URB's payload in a kworker and only resubmits the URB after the
# copy. With 5 URBs of ~32 KB there is ~1.5 ms of host-side buffering. When the app loads the
# CPUs, those kworkers (SCHED_OTHER, nice -20) wait behind other work, the camera's FIFO
# overruns, and it sets the UVC error bit. uvcvideo drops that frame, and about 3% of the time
# the camera resyncs 2048 bytes off and the next full-size frame is rotated.
#
# Fix: SCHED_FIFO 50 for those kworkers. Measured 2026-09-22 on handheld-autonomy, app loaded
# after /command/reinit_field:
#   default     error-bit frames 1322 in 150 s, rotated 26/3559 (0.73%), runner 22 Hz
#   this fix    error-bit frames 1 in 300 s,    rotated 0/9234,          runner 30 Hz
# The frame rate recovers too: the error-bit frames were ~25% of the stream, silently dropped.
#
# The workers are created lazily and pooled, so the keeper polls once a second. A worker keeps
# its policy after the camera closes, so it only ever has to catch new ones. Harmless on a box
# with no UVC camera.

install_uvcvideo_rt() {
    local service_name="auto-battlebot-uvcvideo-rt.service"
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
