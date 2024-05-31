Based on the wonderfull work of [Velocet/iown-homecontrol](https://github.com/Velocet/iown-homecontrol)

### **Disclaimer**  
Tool designed for educational and testing purposes, provided "as is", without warranty of any kind. Creators and contributors are not responsible for any misuse or damage caused by this tool.

_I don't give any support, especially concerning the obsolete 1W part._

### Usage
Use it like a scanner, do some commands on your real device to identify the corresponding CMDid and address

-> Choose you board before build.

### Brief explanation[^1]:
Use 2 interrupts 
  - if PAYLOAD and RX -> decode the frame [^3]
  - if PAYLOAD and TX_READY -> send the frame, decode it [^3]

### platformio[^2] :
_At first time or when a JSON in data folder is modified:_
  1. build filesystem image
  2. upload filesystem image
     
_After any other changes:_  
  - upload and monitor

[^1]: I use an SX1276. If CC1101/SX1262: Feel free to use the old code (Not checked/garanted)

[^2]: I use Visual Studio Code Insider

[^3]: Decoding can be verbose (RSSI, Timing, ...)

#### **License**

[Creative Commons Attribution-NonCommercial-NoDerivs (CC-BY-NC-ND)](http://creativecommons.org/licenses/by-nc-nd/4.0/)
This only allows people to download and share your work for no commercial gain and for no other purposes.