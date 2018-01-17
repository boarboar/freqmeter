

#ifndef FIXFFT_H
#define FIXFFT_H

#ifdef ARDUINO
	#if ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h" /* This is where the standard Arduino code lies */
	#endif
#else
	#include <stdlib.h>
	#include <stdio.h>
	#include <avr/io.h>
	#include <math.h>
	#include "defs.h"
	#include "types.h"
#endif

//#define N_WAVE      1024    /* full length of Sinewave[] */
//#define LOG2_N_WAVE 10      /* log2(N_WAVE) */

//#define N_WAVE      32    /* full length of Sinewave[] */
//#define LOG2_N_WAVE 5      /* log2(N_WAVE) */

#define N_WAVE      128    /* full length of Sinewave[] */
#define LOG2_N_WAVE 7      /* log2(N_WAVE) */

/* Custom constants */
#define FFT_FORWARD 0x01
#define FFT_REVERSE 0x00
/* Windowing type */
#define FFT_WIN_TYP_RECTANGLE 0x00 /* rectangle (Box car) */
#define FFT_WIN_TYP_HAMMING 0x01 /* hamming */
#define FFT_WIN_TYP_HANN 0x02 /* hann */
#define FFT_WIN_TYP_TRIANGLE 0x03 /* triangle (Bartlett) */
#define FFT_WIN_TYP_BLACKMAN 0x04 /* blackmann */
#define FFT_WIN_TYP_FLT_TOP 0x05 /* flat top */
#define FFT_WIN_TYP_WELCH 0x06 /* welch */
/*Mathematial constants*/
#define twoPi 6.28318531
#define fourPi 12.56637061

/*
 fix_fft() - perform forward/inverse fast Fourier transform.
 fr[n],fi[n] are real and imaginary arrays, both INPUT AND
 RESULT (in-place FFT), with 0 <= n < 2**m; set inverse to
 0 for forward transform (FFT), or 1 for iFFT.
*/
int16_t fix_fft(int16_t fr[], int16_t fi[], int16_t m, int16_t inverse);
void fix_fft_cp2m(int16_t *vReal, int16_t *vImag, uint16_t samples);
void  fix_fft_debias(int16_t *pdSamples, int8_t n);
void fix_fft_log(int16_t *pdSamples, int8_t n);
void fix_fft_wnd(int16_t *vData, uint16_t samples, uint8_t windowType=FFT_WIN_TYP_HANN, uint8_t dir=FFT_FORWARD);



/*
 fix_fftr() - forward/inverse FFT on array of real numbers.
 Real FFT/iFFT using half-size complex FFT by distributing
 even/odd samples into real/imaginary arrays respectively.
 In order to save data space (i.e. to avoid two arrays, one
 for real, one for imaginary samples), we proceed in the
 following two steps: a) samples are rearranged in the real
 array so that all even samples are in places 0-(N/2-1) and
 all imaginary samples in places (N/2)-(N-1), and b) fix_fft
 is called with fr and fi pointing to index 0 and index N/2
 respectively in the original array. The above guarantees
 that fix_fft "sees" consecutive real samples as alternating
 real and imaginary samples in the complex array.
*/
//int fix_fftr(char f[], int m, int inverse);




#endif