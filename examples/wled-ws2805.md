# QuinLED dig2analog-plus-5ch with WLED

In WS mode the board behaves like a WS2805: one board is one "LED" with five channels.

1. **LED type:** WS2805 (5 channels: red, green, blue, warm white, cold white). The board's terminals CH1-CH5 are R,
   G, B, W1 (warm), W2 (cold), the same order as WLED sends.
2. **Length:** the number of boards on the data line.
3. **Color order:** RGB.
4. **White:** set WLED's white management as for any RGBCCT strip. WLED treats each board as one RGBCCT pixel;
   for five separate single-colour lights per board, use ESPHome instead.
5. **Data pin:** the board's Data-In; chained boards follow on its Data-Out.

The board detects the protocol by itself and starts with the one it used last. If it was set to a fixed mode
(e.g. Ultra) before, switch it back to auto or WS2805 first: with ESPHome's `ws2915.send_config` (command
`protocol`, value 0 or 2; see `esphome-ws-mode-config-frames.yaml`).

Board settings (PWM frequency, dithering, power-on scene, signal loss, ...) can't be sent from WLED. Set them once
with ESPHome; the board stores them and keeps them when WLED drives it. The settings and their
defaults: [WS modes and config frames](https://github.com/intermittech/quinled-ultra-firmware/wiki/WS-Modes-And-Config-Frames).

The WS2805 is 8 bit per channel. The board turns that into smooth 16-bit output (gamma option, inter-frame
smoothing, dithering at its high PWM frequency). For 16 bit end to end, use WS2915 or Ultra mode from ESPHome.
