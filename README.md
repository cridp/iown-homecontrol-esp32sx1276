Pairing process NOT included, work by yourself.

### Usage

Use it like a scanner, do some commands on your real device to identify the corresponding CMDid and address

-> M~~odify Pins in board_config.h according your board~~
Not needed if you choose you board before build.

### Brief explanation[^1]:

Use 2 interrupts for:
  - PREAMBLE
  - PAYLOAD

Use a 100 µs timer to check state of interrupts & flags (iohcRadio::tickerCounter)

**WIP : Removing the 100µs TickTimer Handler and use Interrupt Handler (faster)**
  - if PAYLOAD and RX -> decode the frame [^3]
  - if PAYLOAD and TX_READY -> send the frame, decode it [^3]

**WIP : Using a better SPI driver**

**WIP : Learning Mode**

### platformio[^2] :
_At first time or when a JSON in data folder is modified:_
  1. build filesystem image
  2. upload filesystem image
     
_After any other changes:_  
  - upload and monitor

[^1]: I use SX1276. If CC1101/SX1262: Feel free to use the old code (Not checked/garanted)
[^2]: I use Visual Studio Code Insider / CLion Nova
[^3]: Decoding can be verbose (RSSI, Timing, ...)
