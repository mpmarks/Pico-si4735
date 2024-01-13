#ifndef SIMPLE_DIGITAL_H
#define SIMPLE_DIGITAL_H

// Tuning parameters for USB Audio -> transmit frequency code
#define FSK_TOUT             50 //FSK timeout in mSecs (no signal present for more than)
#define SAMPLE_RATE       48000 //Audio sample rate
#define FSK_MIN           20000 //Minimum frequency allowed (200 Hz)
#define FSK_MAX          300000 //Maximum frequency allowed (3000 Hz)
#define FSK_THRESHOLD         5 //Minimum threshold to change the frequency
#define FSK_MIN_CHANGE       10 //Minimum frequency change (0.10 Hz) in order to avoid small measurement errors to produce frequency shifts


#endif