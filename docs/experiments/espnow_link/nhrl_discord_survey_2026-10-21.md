# ESP-NOW reliability at NHRL: Discord survey

I searched the NHRL Discord on 2026-10-21 for reports of ESP-NOW in competition. Nobody reports
running ESP-NOW at NHRL without trouble. The two first-hand robot accounts both hit problems at
the venue that never showed up at home. ELRS is the consensus control link.

Conclusion for Mr Stabs Mk2: keep CRSF/ELRS as the control authority and leave ESP-NOW as
shadow and telemetry.

## Method

- Searched Discord in the browser for `espnow`, `"esp-now"`, `esp32 receiver`,
  `wifi interference`, `2.4ghz interference`, `dropped packets`, `bluetooth disconnect`.
- About 80 hits from 2022 to 2026. I read the surrounding conversation for the 8 most relevant.
- Discord's search tokenizes `esp-now` and `esp now` the same way (43 hits). `espnow` is a
  separate token (18 hits).
- All links below are in the NHRL server (guild `651601084019900483`).

## First-hand reports

### beb: ESP-NOW ping test at NHRL, January 2025

[#3lbs](https://discord.com/channels/651601084019900483/663805006226391080/1333378466992885802)

- Two ESP32-S3s pinged each other over ESP-NOW, tested morning, peak hours, and evening.
- Morning and evening were fine.
- At peak hours there were "far too many dropped packets to be useful", with the devices
  overwhelmed within 1 m of each other.
- beb offered to post the recorded data. I did not find it.

### Pearl (Injection!): ESP-NOW as the control link

Setup: Steam Deck, USB serial, ESP32 dongle, ESP-NOW, ESP32-S3 on the robot. 20 Hz at first,
with 50 Hz each way as the target. Pearl planned to tolerate up to 90% packet loss for control.

- 2025-02-25: "experimenting with ESP-NOW was a mistake". Pearl ordered a traditional RX/TX as
  a backup before the March event
  ([#mar-1-2025](https://discord.com/channels/651601084019900483/1320874580771868694/1343981797821644873)).
- 2025-03-02 event recap: "I actually got ESP-NOW working in a combat environment."
- 2025-05-01: "Reliability ESP-NOW was a mistake, I should've stuck with Bluetooth"
  ([#bot-building-general](https://discord.com/channels/651601084019900483/810932842564943913/1367504554111008891)).
- 2026-09-27: switching to "something based on a stripped down ELRS" for the gen2 electronics
  ([#bot-building-general](https://discord.com/channels/651601084019900483/810932842564943913/1553879369855008900)).

The 2026-09-27 thread explains the failure:

- Issue: at NHRL in early 2025, latency in the test box reached about 1 minute. In a couple of
  matches it crept to about 3 s by the end. It did not happen at home or at other venues.
- Cause: congestion at NHRL, plus packets backing up and trickling out on the Steam Deck side.
- Fix: a software change that let the comms drop packets without building delay.
- After the fix, end-to-end latency was still about 100 ms. Pearl measured it by filming a
  button press and an LED on the robot at high frame rate.

### Arena Timer wireless remotes, July to October 2026

Vinnie Marco's
["Arena Timers" thread](https://discord.com/channels/651601084019900483/1507669881691181126/1531682566883508286).
The remotes are ESP8266s sending ESP-NOW to an ESP32 timer. Several event organizers use them.

- A ref had connection issues with a remote about 2 ft from the timer.
- Honey Cracked saw lag and delay. The remotes stopped working behind wood bumpers or more than a
  few layers of polycarbonate.
- Vinnie: "It was so reliable and good in all my testing." He tested at home, which was not
  representative of a venue.
- Miss mechanism: the remote broadcasts on each channel in turn. If the timer changes WiFi
  channel mid-burst it can miss the packet.
- Fixes:
  - Firmware v2.0.0 stops the ESP32 WiFi core from sleeping, because sleep made it miss
    remote packets.
  - New remotes on an STM32 with 915 MHz LoRa, field tested 2026-08-22.

## Community opinion

- RealNewDeal (Gigabyte), 2024-11-28: "The ESPnow system has huge issues with wifi interference"
  ([link](https://discord.com/channels/651601084019900483/810932842564943913/1311580778743992343)).
- RealNewDeal, 2024-10-18: worried a stray WiFi packet could be read as an ESP-NOW command, and
  "you lose connection you lose the match"
  ([#battlebots](https://discord.com/channels/651601084019900483/809971550937808926/1296651534754648154)).
- fuzzhao: ESP-NOW is built for low-power single transmissions on one channel. RC radios
  transmit continuously and frequency-hop. fuzzhao tried hopping on top of ESP-NOW, but
  Espressif's channel-switch API is not real time and the hops landed at the wrong times.
- Benji (Hex/Swarm): uses ESP-NOW for cheap swarm robots, wants proper channel hopping, and
  runs LoRa in their plants.
- 2.4 GHz trouble at NHRL also hits FS2A receivers, mostly blamed on bad batches. The usual
  advice is "just pull the bandaid and run ELRS"
  ([link](https://discord.com/channels/651601084019900483/810932842564943913/1336475435789193217)).

## How our link compares

| Failure seen at NHRL | Our firmware |
| --- | --- |
| Command backlog (Pearl, 1 min latency) | Robot drains the whole RX queue every tick (`firmware/mr_stabs_mk2/src/espnow_link.cpp:140`). Dongle drops serial frames to the host instead of blocking (`firmware/espnow_dongle/src/main.cpp:165`). |
| WiFi sleep missing packets (Arena Timer) | Dongle sets `WIFI_PS_NONE` (`firmware/espnow_dongle/src/main.cpp:351`). Robot runs a pinned SoftAP. |
| Peak-hour packet loss (beb) | Not tested. Every test so far has been at home or on the bench. |
| Channel mismatch (Arena Timer) | Robot pins the channel through the SoftAP and tracks `last_good_channel_`. |

I have not checked the host-to-dongle direction. If the Jetson writes faster than the radio
sends, frames could pile up in the dongle's USB RX buffer or ESP-NOW's send queue, which would
produce the same delay Pearl saw.

## Next steps

- Keep CRSF/ELRS as the control authority. Only promote ESP-NOW to control after it holds up at
  NHRL peak hours.
- Log `rx_queue_full`, send history, and RSSI per match to the MCAP so the first event gives
  real numbers.
- Run a test at a venue during peak hours, the way beb did, before trusting the link.
- Check the host-to-dongle path for stale-frame buildup. Send only the latest command.
- Ask beb in #3lbs for the peak-hour packet-loss data.
