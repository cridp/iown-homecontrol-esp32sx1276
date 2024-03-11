### Brief explanation[^1]:

Use 2 interrupts for:
  - PREAMBLE
  - PAYLOAD

Use a 100 µs timer to check state of interrupts (iohcRadio::tickerCounter)

    - if PAYLOAD and RX -> decode the frame [^3]
    - if PAYLOAD and TX_READY -> send the frame, decode it [^3]
    
Modify Pins in board_config.h according your board
### 1W ? Not my cup of tea, but implemented 20 real devices to play with.
### 2W howTo

### platformio[^2] :
_At first time or when a JSON in data folder is modified:_
  1. build filesystem image
  2. upload filesystem image
     
_After any other changes:_  
  - upload and monitor

[^1]: I use SX1276. If CC1101: Feel free to use the old code (Not checked/garanted)
[^2]: I use Visual Studio Code Insider / CLion Nova
[^3]: Decoding can be verbose, but slow down the process.