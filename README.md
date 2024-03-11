### Brief explanation[^1]:

Use 2 interrupts for:
  - PREAMBLE
  - PAYLOAD
Use a 100 µs timer to check state of interrupts (iohcRadio::tickerCounter)
    go to RX
    - if PAYLOAD and RX -> decode the frame (opt)
    - if PAYLOAD and TX_READY -> send the frame, decode it (opt)
    
Modify Pins in board_config.h according your board

### platformio[^2] :
_At first time or when a JSON in data folder is modified:_
  1. build filesystem image
  2. upload filesystem image
     
_After any other changes:_  
  - upload and monitor

[^1]: I use SX1276. If CC1101 it use the old code (Not checked/garanted)
[^2]: I use : Visual Studio Code Insider / CLion Nova