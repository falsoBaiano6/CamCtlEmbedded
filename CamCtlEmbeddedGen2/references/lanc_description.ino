/* In the standard 8-byte LANC packet frame, an external controller (acting as a 
slave/remote relative to the camera's master timing) injects command data 
exclusively into Byte 0 and Byte 1.
Packet Structure and Injection Slots
Byte 0: First command slot (e.g., zoom/focus or record start/stop commands).
Byte 1: Second command slot (extended command or secondary parameters).
Bytes 2 & 3: Reserved for tuners or extra control/command extensions 
(usually left as 00 by standard controllers).
Bytes 4 through 7: Camera-to-controller VCR status bytes, counter data, and 
device status (driven strictly by the camera master).
How Injection Works
The camera (master) generates the clock and pulls the line low to signal the 
start bit for each of the 8 bytes in a frame.
The external device (slave/remote) listens for the start bit of Byte 0, waits 
for the precise bit timing, and pulls the open-collector line low to inject 
its command bits into Byte 0 and immediately after during Byte 1.
For the remaining bytes (Bytes 2–7), the external controller stops driving the 
line and only listens to the status information transmitted by the camera. */