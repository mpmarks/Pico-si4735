# Pico-si4735
RPI Pico (Waveshare Pico Zero) with si4732-A10 radio experiment. In development for a very simple FT8 transceiver.

My goal is to create an ADX style digital modes transceiver with the minimum of chips and complexity. 
The first step is using an inexpensive Waveshare Pico module for both CAT Serial and USB Audio. The
code for this is derived from the JA1RAV code but I added DMA for the ADC to reduce the CPU 
loading. Using a Si4732-A10 receiver chip simplifies that part of the design and provides DSP functions
without extra hardware or time critical firmware.

The Si5351 generates the 32769 Hz clock for the Si4732 (CLK1) as well as the transmit clock (CLK0). I'm going
to try using CLK2 to calibrate the receiver.

Status: 1/14/24. Receiver works well for 20M FT8 sending the received audio via USB to WSJT-X.
The FT8 transmit code depends on determining the audio frequency
from the incoming USB Audio. This code is still in development. There are 2 problems currently 
the derived audio frequency is not correct, and there are short gaps in the transmit stream that may be 
due to insuffient time in the transmit thread. Under investigation.


