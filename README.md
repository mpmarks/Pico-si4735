# Pico-si4735
RPI Pico (Waveshare Pico Zero) with si4732-d10 radio experiment. In development for a very simple FT8 transceiver.

My goal is to create an ADX style digital transceiver with the minimum of chips and complexity. 
The first step is using an inexpensive Waveshare Pico module for Serial and USB Audio. The
code for this is derived from the JA1RAV code but I added DMA for the ADC to reduce the CPU 
loading. Using a Si4732-D10 receiver chip simplifies that part of the design and provides DSP functions
without extra hardware or software.


