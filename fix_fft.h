

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

#define N_WAVE      64    /* full length of Sinewave[] */
#define LOG2_N_WAVE 6      /* log2(N_WAVE) */

/*
 fix_fft() - perform forward/inverse fast Fourier transform.
 fr[n],fi[n] are real and imaginary arrays, both INPUT AND
 RESULT (in-place FFT), with 0 <= n < 2**m; set inverse to
 0 for forward transform (FFT), or 1 for iFFT.
*/
int16_t fix_fft(int16_t fr[], int16_t fi[], int16_t m, int16_t inverse);
void fix_fft_cp2m(int16_t *vReal, int16_t *vImag, uint16_t samples);



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