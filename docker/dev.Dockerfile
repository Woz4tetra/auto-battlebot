# Personal devtools layer on top of the playback toolchain image.
#
# Adds https://github.com/aalbaali/workstation_setup ("Quick bootstrap": ansible, then
# playbooks for base CLI tools, zsh + oh-my-zsh + starship, neovim + packer, vim) into
# /home/dev. Kept out of docker/playback.Dockerfile on purpose: that image is the
# team-shared toolchain (docs/docker_playback.md), and personal dotfiles have no business
# in it. This is an opt-in layer built and run separately -- see
# scripts/docker/build_dev_image.sh and scripts/docker/dev_shell.sh.
#
# docker-compose.playback.yml's `dev` service runs this the same way it runs `playback`:
# arbitrary host UID via --user, with /etc/passwd bind-mounted read-only and HOME forced
# to /home/dev. That means the identity this Dockerfile builds under (the `dev` user
# below) is never the identity the container actually runs as -- it only exists so
# ansible's `{{ ansible_user_id }}`-derived paths resolve to /home/dev at build time
# instead of /home/root. Whatever ends up on disk there has to be world-writable so the
# real runtime UID can use and edit it, same as playback.Dockerfile does for
# /usr/local/zed and /opt/venv.
ARG BASE_IMAGE=auto-battlebot-playback:latest
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN groupadd --gid 1500 dev \
    && useradd --uid 1500 --gid 1500 -m -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev \
    && chmod 0440 /etc/sudoers.d/dev

COPY docker/fix_workstation_setup_vars.py /tmp/fix_workstation_setup_vars.py

USER dev
WORKDIR /home/dev

# We don't just pipe the repo's clone_and_run_dev_playbook one-liner directly: as of this
# writing, its ansible/tasks/*.yaml use the old list-style `vars:` block (`- home: ...`),
# which current ansible-core (pulled from ppa:ansible/ansible) rejects outright ("Vars in
# a Play must be specified as a dictionary"). fix_workstation_setup_vars.py merges that
# into a dict and otherwise runs the exact same playbooks the bootstrap script does. Safe
# to go back to the one-liner once the upstream YAML is fixed.
#
# setup_nvim's "Install nvim packages" task runs `timeout 30 nvim --headless ...
# PackerSync`, cloning a dozen-plus plugins from cold in the time budgeted for a warm
# cache. It (and setup_vim's plugin install) can legitimately blow past 30s on a fresh
# image and fail the whole play; that shouldn't fail the image build over what finishes
# itself on the next interactive nvim/vim launch, so those two are allowed to fail here.
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        software-properties-common python3-yaml \
    && sudo add-apt-repository --yes --update ppa:ansible/ansible \
    && sudo apt-get install -y --no-install-recommends ansible \
    && sudo rm -rf /var/lib/apt/lists/* \
    && git clone https://github.com/aalbaali/workstation_setup.git /tmp/workstation_setup --depth 1 \
    && python3 /tmp/fix_workstation_setup_vars.py "/tmp/workstation_setup/ansible/tasks/*.yaml" \
    && cd /tmp/workstation_setup/ansible \
    && ansible-playbook tasks/install_minimum_packages.yaml \
    && ansible-playbook tasks/setup_zsh.yaml \
    && (ansible-playbook tasks/setup_nvim.yaml \
        || echo "warning: nvim plugin sync did not finish inside its 30s budget on a cold cache -- run :PackerSync inside nvim to finish it") \
    && (ansible-playbook tasks/setup_vim.yaml \
        || echo "warning: vim plugin install did not finish -- run vim +PlugInstall to finish it") \
    && cd /home/dev \
    && sudo rm -rf /tmp/workstation_setup /tmp/fix_workstation_setup_vars.py \
    && sudo chsh -s "$(command -v zsh)" dev \
    && sudo chown -R dev:dev /home/dev \
    && chmod -R a+rwX /home/dev

# The `dev` user above only exists for build-time paths (see note atop this file); the
# container actually runs as an arbitrary host UID via --user, resolved to a name
# through the read-only /etc/passwd mount. That UID has no matching sudoers entry, so
# the "dev ALL=(ALL) NOPASSWD:ALL" rule earlier in this file never applies to it and
# sudo fails. Match by ALL instead of by name, the same "must work for whatever UID
# shows up at runtime" treatment this file already gives /home/dev, /usr/local/zed, and
# /opt/venv. Kept as its own layer so it doesn't invalidate the ansible/devtools layer
# above.
RUN echo "ALL ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/nopasswd-all >/dev/null \
    && sudo chmod 0440 /etc/sudoers.d/nopasswd-all

# The sudoers rule above still isn't enough: only /etc/passwd and /etc/group are
# bind-mounted from the host, not /etc/shadow, so PAM's account phase
# (/etc/pam.d/sudo -> @include common-account -> pam_unix.so) finds no shadow entry for
# the runtime UID, falls through to the `pam_deny.so requisite` fallback in
# common-account, and sudo reports "account validation failure, is your account
# locked?" even though the sudoers rule matched. There's no meaningful account to
# validate here (the NOPASSWD:ALL rule above already trusts any resolved user), so
# replace sudo's account check with an unconditional pass. Scoped to /etc/pam.d/sudo
# specifically rather than editing the shared common-account file.
RUN sudo sed -i 's/^@include common-account$/account required pam_permit.so/' /etc/pam.d/sudo

WORKDIR /workspace
CMD ["/bin/zsh"]
