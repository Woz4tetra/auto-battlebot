# Host dependencies

This guide assumes you're using a linux machine to run all these commands. You'll have to adapt these instructions if you're on another OS.

I recommend running all of these commands inside of a tmux session in case of network dropouts:

-   Create tmux session: `tmux new -s build`
-   Hide tmux session: ctrl-B D
-   Reattach tmux session: `tmux a -t build`

## Jetson initial setup

[Follow this guide from NVidia](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

Setup options:

-   username: ben
-   password: s0mething

After rebooting, try ssh: `ssh ben@ubuntu.local` <br>
Install openssh server if it doesn’t work: `sudo apt-get install openssh-server -y`

## Apt refresh + basic packages

-   `sudo apt update`
-   `sudo apt upgrade`
-   `sudo apt autoremove`
-   `sudo apt install nano tmux curl htop`
-   `sudo reboot`

## SSH setup

Generate an SSH key

`ssh-keygen`

Press enter for all default options

-   `cd ~/.ssh/`
-   `cp id_rsa.pub authorized_keys`
-   On your machine:
    -   `scp ben@ubuntu:/home/ben/.ssh/id_rsa ~/.ssh/auto_battlebot_id_rsa`
    -   `scp ben@ubuntu:/home/ben/.ssh/id_rsa.pub ~/.ssh/auto_battlebot_id_rsa.pub`
-   Optionally disable password login:
    -   `sudo nano /etc/ssh/sshd_config`
    -   Search for `#PasswordAuthentication yes`
    -   Change to `PasswordAuthentication no`
    -   Save and exit (ctrl-S ctrl-X)
-   `sudo service ssh restart`
-   Find the Jetson's IP with `ifconfig`
-   Try to login with `ssh -i ~/.ssh/auto_battlebot_id_rsa ben@ubuntu`

## Disable wifi power saving

[Based on this guide](https://unix.stackexchange.com/questions/269661/how-to-turn-off-wireless-power-management-permanently)

-   `sudo nano /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`
-   Change:

```
[connection]
wifi.powersave = 3
```

to

```
[connection]
wifi.powersave = 2
```

-   `sudo systemctl restart network` This will kill your SSH session. Alternatively, you can run `sudo reboot`
-   Have a display and keyboard on hand in case this goes wrong.

## Disable desktop

This reduces boot times by ~15 seconds.

https://forums.developer.nvidia.com/t/how-to-boot-jetson-nano-in-text-mode/73636

-   `sudo systemctl set-default multi-user.target`

To re-enable the desktop:

-   `sudo systemctl set-default graphical.target`

To start the desktop manually after logging into the CLI:

-   `sudo systemctl start gdm3.service`

## Add to sudo group

-   `sudo usermod -aG sudo $USER`
-   `sudo visudo`
-   If vim opens,
    -   Press a to enter edit mode
    -   Add the following line to the file: <br>`ben  ALL=(ALL) NOPASSWD:ALL`
    -   Press esc
    -   Type `:x`
    -   Press enter
-   If nano opens,
    -   Add the following line to the file: <br>`ben  ALL=(ALL) NOPASSWD:ALL`
    -   Press ctrl-S then ctrl-X
-   Log out with ctrl-D
-   Log in and try `sudo su`
-   If no password prompt appears, these steps worked

## Increase memory security limit

This increases the maximum RAM a process from a given user can use.

-   `sudo nano /etc/security/limits.conf`
-   File should look like this (increase all limits to 8GB):
    ```
    ben hard stack 8192
    ben soft stack 8192
    ubuntu hard stack 8192
    ubuntu soft stack 8192
    root hard stack 8192
    root soft stack 8192
    ```
