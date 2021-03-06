/* fix_fft.c - Fixed-point in-place Fast Fourier Transform  */
/*
  All data are fixed-point short integers, in which -32768
  to +32768 represent -1.0 to +1.0 respectively. Integer
  arithmetic is used for speed, instead of the more natural
  floating-point.

  For the forward FFT (time -> freq), fixed scaling is
  performed to prevent arithmetic overflow, and to map a 0dB
  sine/cosine wave (i.e. amplitude = 32767) to two -6dB freq
  coefficients. The return value is always 0.

  For the inverse FFT (freq -> time), fixed scaling cannot be
  done, as two 0dB coefficients would sum to a peak amplitude
  of 64K, overflowing the 32k range of the fixed-point integers.
  Thus, the fix_fft() routine performs variable scaling, and
  returns a value which is the number of bits LEFT by which
  the output must be shifted to get the actual amplitude
  (i.e. if fix_fft() returns 3, each value of fr[] and fi[]
  must be multiplied by 8 (2**3) for proper scaling.
  Clearly, this cannot be done within fixed-point short
  integers. In practice, if the result is to be used as a
  filter, the scale_shift can usually be ignored, as the
  result will be approximately correctly normalized as is.

  Written by:  Tom Roberts  11/8/89
  Made portable:  Malcolm Slaney 12/15/94 malcolm@interval.com
  Enhanced:  Dimitrios P. Bouras  14 Jun 2006 dbouras@ieee.org
*/

#include "fix_fft.h"

/*
  Henceforth "short" implies 16-bit word. If this is not
  the case in your architecture, please replace "short"
  with a type definition which *is* a 16-bit word.
*/

/*
  Since we only use 3/4 of N_WAVE, we define only
  this many samples, in order to conserve data space.
*/
const int16_t Sinewave[N_WAVE-N_WAVE/4] = 
#if (N_WAVE == 8)
{0,23170,32767,23170,0,-23170};

#elif (N_WAVE == 16)
{0,12539,23170,30273,32767,30273,23170,12539,0,-12539,-23170,-30273};

#elif (N_WAVE == 32)
{0,6393,12539,18204,23170,27245,30273,32137,32767,32137,30273,27245,23170,18204,12539,6393,0,-6393,-12539,-18204,-23170,
-27245,-30273,-32137};

#elif (N_WAVE == 64)
{0,3212,6393,9512,12539,15446,18204,20787,23170,25329,27245,28898,30273,31356,32137,32609,32767,32609,32137,31356,30273,
28898,27245,25329,23170,20787,18204,15446,12539,9512,6393,3212,0,-3212,-6393,-9512,-12539,-15446,-18204,-20787,-23170,
-25329,-27245,-28898,-30273,-31356,-32137,-32609};

#elif (N_WAVE == 128)
{0,1608,3212,4808,6393,7962,9512,11039,12539,14010,15446,16846,18204,19519,20787,22005,23170,24279,25329,26319,27245,28105,
28898,29621,30273,30852,31356,31785,32137,32412,32609,32728,32767,32728,32609,32412,32137,31785,31356,30852,30273,29621,
28898,28105,27245,26319,25329,24279,23170,22005,20787,19519,18204,16846,15446,14010,12539,11039,9512,7962,6393,4808,3212,
1608,0,-1608,-3212,-4808,-6393,-7962,-9512,-11039,-12539,-14010,-15446,-16846,-18204,-19519,-20787,-22005,-23170,-24279,
-25329,-26319,-27245,-28105,-28898,-29621,-30273,-30852,-31356,-31785,-32137,-32412,-32609,-32728};

#elif (N_WAVE == 256)
{0,804,1608,2410,3212,4011,4808,5602,6393,7179,7962,8739,9512,10278,11039,11793,12539,13279,14010,14732,15446,16151,
16846,17530,18204,18868,19519,20159,20787,21403,22005,22594,23170,23731,24279,24811,25329,25832,26319,26790,27245,
27683,28105,28510,28898,29268,29621,29956,30273,30571,30852,31113,31356,31580,31785,31971,32137,32285,32412,32521,
32609,32678,32728,32757,32767,32757,32728,32678,32609,32521,32412,32285,32137,31971,31785,31580,31356,31113,30852,
30571,30273,29956,29621,29268,28898,28510,28105,27683,27245,26790,26319,25832,25329,24811,24279,23731,23170,22594,
22005,21403,20787,20159,19519,18868,18204,17530,16846,16151,15446,14732,14010,13279,12539,11793,11039,10278,9512,
8739,7962,7179,6393,5602,4808,4011,3212,2410,1608,804,0,-804,-1608,-2410,-3212,-4011,-4808,-5602,-6393,-7179,-7962,
-8739,-9512,-10278,-11039,-11793,-12539,-13279,-14010,-14732,-15446,-16151,-16846,-17530,-18204,-18868,-19519,-20159,
-20787,-21403,-22005,-22594,-23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,-27245,-27683,-28105,-28510,-28898,
-29268,-29621,-29956,-30273,-30571,-30852,-31113,-31356,-31580,-31785,-31971,-32137,-32285,-32412,-32521,-32609,-32678,
-32728,-32757};

#elif (N_WAVE == 512)
{0,402,804,1206,1608,2009,2410,2811,3212,3612,4011,4410,4808,5205,5602,5998,6393,6786,7179,7571,7962,8351,8739,9126,
9512,9896,10278,10659,11039,11417,11793,12167,12539,12910,13279,13645,14010,14372,14732,15090,15446,15800,16151,16499,
16846,17189,17530,17869,18204,18537,18868,19195,19519,19841,20159,20475,20787,21096,21403,21705,22005,22301,22594,22884,
23170,23452,23731,24007,24279,24547,24811,25072,25329,25582,25832,26077,26319,26556,26790,27019,27245,27466,27683,27896,
28105,28310,28510,28706,28898,29085,29268,29447,29621,29791,29956,30117,30273,30424,30571,30714,30852,30985,31113,31237,
31356,31470,31580,31685,31785,31880,31971,32057,32137,32213,32285,32351,32412,32469,32521,32567,32609,32646,32678,32705,
32728,32745,32757,32765,32767,32765,32757,32745,32728,32705,32678,32646,32609,32567,32521,32469,32412,32351,32285,32213,
32137,32057,31971,31880,31785,31685,31580,31470,31356,31237,31113,30985,30852,30714,30571,30424,30273,30117,29956,29791,
29621,29447,29268,29085,28898,28706,28510,28310,28105,27896,27683,27466,27245,27019,26790,26556,26319,26077,25832,25582,
25329,25072,24811,24547,24279,24007,23731,23452,23170,22884,22594,22301,22005,21705,21403,21096,20787,20475,20159,19841,
19519,19195,18868,18537,18204,17869,17530,17189,16846,16499,16151,15800,15446,15090,14732,14372,14010,13645,13279,12910,
12539,12167,11793,11417,11039,10659,10278,9896,9512,9126,8739,8351,7962,7571,7179,6786,6393,5998,5602,5205,4808,4410,4011,
3612,3212,2811,2410,2009,1608,1206,804,402,0,-402,-804,-1206,-1608,-2009,-2410,-2811,-3212,-3612,-4011,-4410,-4808,-5205,
-5602,-5998,-6393,-6786,-7179,-7571,-7962,-8351,-8739,-9126,-9512,-9896,-10278,-10659,-11039,-11417,-11793,-12167,-12539,
-12910,-13279,-13645,-14010,-14372,-14732,-15090,-15446,-15800,-16151,-16499,-16846,-17189,-17530,-17869,-18204,-18537,
-18868,-19195,-19519,-19841,-20159,-20475,-20787,-21096,-21403,-21705,-22005,-22301,-22594,-22884,-23170,-23452,-23731,
-24007,-24279,-24547,-24811,-25072,-25329,-25582,-25832,-26077,-26319,-26556,-26790,-27019,-27245,-27466,-27683,-27896,
-28105,-28310,-28510,-28706,-28898,-29085,-29268,-29447,-29621,-29791,-29956,-30117,-30273,-30424,-30571,-30714,-30852,
-30985,-31113,-31237,-31356,-31470,-31580,-31685,-31785,-31880,-31971,-32057,-32137,-32213,-32285,-32351,-32412,-32469,
-32521,-32567,-32609,-32646,-32678,-32705,-32728,-32745,-32757,-32765};

#elif (N_WAVE == 1024)
{
      0,    201,    402,    603,    804,   1005,   1206,   1406,
   1607,   1808,   2009,   2209,   2410,   2610,   2811,   3011,
   3211,   3411,   3611,   3811,   4011,   4210,   4409,   4608,
   4807,   5006,   5205,   5403,   5601,   5799,   5997,   6195,
   6392,   6589,   6786,   6982,   7179,   7375,   7571,   7766,
   7961,   8156,   8351,   8545,   8739,   8932,   9126,   9319,
   9511,   9703,   9895,  10087,  10278,  10469,  10659,  10849,
  11038,  11227,  11416,  11604,  11792,  11980,  12166,  12353,
  12539,  12724,  12909,  13094,  13278,  13462,  13645,  13827,
  14009,  14191,  14372,  14552,  14732,  14911,  15090,  15268,
  15446,  15623,  15799,  15975,  16150,  16325,  16499,  16672,
  16845,  17017,  17189,  17360,  17530,  17699,  17868,  18036,
  18204,  18371,  18537,  18702,  18867,  19031,  19194,  19357,
  19519,  19680,  19840,  20000,  20159,  20317,  20474,  20631,
  20787,  20942,  21096,  21249,  21402,  21554,  21705,  21855,
  22004,  22153,  22301,  22448,  22594,  22739,  22883,  23027,
  23169,  23311,  23452,  23592,  23731,  23869,  24006,  24143,
  24278,  24413,  24546,  24679,  24811,  24942,  25072,  25201,
  25329,  25456,  25582,  25707,  25831,  25954,  26077,  26198,
  26318,  26437,  26556,  26673,  26789,  26905,  27019,  27132,
  27244,  27355,  27466,  27575,  27683,  27790,  27896,  28001,
  28105,  28208,  28309,  28410,  28510,  28608,  28706,  28802,
  28897,  28992,  29085,  29177,  29268,  29358,  29446,  29534,
  29621,  29706,  29790,  29873,  29955,  30036,  30116,  30195,
  30272,  30349,  30424,  30498,  30571,  30643,  30713,  30783,
  30851,  30918,  30984,  31049,  31113,  31175,  31236,  31297,
  31356,  31413,  31470,  31525,  31580,  31633,  31684,  31735,
  31785,  31833,  31880,  31926,  31970,  32014,  32056,  32097,
  32137,  32176,  32213,  32249,  32284,  32318,  32350,  32382,
  32412,  32441,  32468,  32495,  32520,  32544,  32567,  32588,
  32609,  32628,  32646,  32662,  32678,  32692,  32705,  32717,
  32727,  32736,  32744,  32751,  32757,  32761,  32764,  32766,
  32767,  32766,  32764,  32761,  32757,  32751,  32744,  32736,
  32727,  32717,  32705,  32692,  32678,  32662,  32646,  32628,
  32609,  32588,  32567,  32544,  32520,  32495,  32468,  32441,
  32412,  32382,  32350,  32318,  32284,  32249,  32213,  32176,
  32137,  32097,  32056,  32014,  31970,  31926,  31880,  31833,
  31785,  31735,  31684,  31633,  31580,  31525,  31470,  31413,
  31356,  31297,  31236,  31175,  31113,  31049,  30984,  30918,
  30851,  30783,  30713,  30643,  30571,  30498,  30424,  30349,
  30272,  30195,  30116,  30036,  29955,  29873,  29790,  29706,
  29621,  29534,  29446,  29358,  29268,  29177,  29085,  28992,
  28897,  28802,  28706,  28608,  28510,  28410,  28309,  28208,
  28105,  28001,  27896,  27790,  27683,  27575,  27466,  27355,
  27244,  27132,  27019,  26905,  26789,  26673,  26556,  26437,
  26318,  26198,  26077,  25954,  25831,  25707,  25582,  25456,
  25329,  25201,  25072,  24942,  24811,  24679,  24546,  24413,
  24278,  24143,  24006,  23869,  23731,  23592,  23452,  23311,
  23169,  23027,  22883,  22739,  22594,  22448,  22301,  22153,
  22004,  21855,  21705,  21554,  21402,  21249,  21096,  20942,
  20787,  20631,  20474,  20317,  20159,  20000,  19840,  19680,
  19519,  19357,  19194,  19031,  18867,  18702,  18537,  18371,
  18204,  18036,  17868,  17699,  17530,  17360,  17189,  17017,
  16845,  16672,  16499,  16325,  16150,  15975,  15799,  15623,
  15446,  15268,  15090,  14911,  14732,  14552,  14372,  14191,
  14009,  13827,  13645,  13462,  13278,  13094,  12909,  12724,
  12539,  12353,  12166,  11980,  11792,  11604,  11416,  11227,
  11038,  10849,  10659,  10469,  10278,  10087,   9895,   9703,
   9511,   9319,   9126,   8932,   8739,   8545,   8351,   8156,
   7961,   7766,   7571,   7375,   7179,   6982,   6786,   6589,
   6392,   6195,   5997,   5799,   5601,   5403,   5205,   5006,
   4807,   4608,   4409,   4210,   4011,   3811,   3611,   3411,
   3211,   3011,   2811,   2610,   2410,   2209,   2009,   1808,
   1607,   1406,   1206,   1005,    804,    603,    402,    201,
      0,   -201,   -402,   -603,   -804,  -1005,  -1206,  -1406,
  -1607,  -1808,  -2009,  -2209,  -2410,  -2610,  -2811,  -3011,
  -3211,  -3411,  -3611,  -3811,  -4011,  -4210,  -4409,  -4608,
  -4807,  -5006,  -5205,  -5403,  -5601,  -5799,  -5997,  -6195,
  -6392,  -6589,  -6786,  -6982,  -7179,  -7375,  -7571,  -7766,
  -7961,  -8156,  -8351,  -8545,  -8739,  -8932,  -9126,  -9319,
  -9511,  -9703,  -9895, -10087, -10278, -10469, -10659, -10849,
 -11038, -11227, -11416, -11604, -11792, -11980, -12166, -12353,
 -12539, -12724, -12909, -13094, -13278, -13462, -13645, -13827,
 -14009, -14191, -14372, -14552, -14732, -14911, -15090, -15268,
 -15446, -15623, -15799, -15975, -16150, -16325, -16499, -16672,
 -16845, -17017, -17189, -17360, -17530, -17699, -17868, -18036,
 -18204, -18371, -18537, -18702, -18867, -19031, -19194, -19357,
 -19519, -19680, -19840, -20000, -20159, -20317, -20474, -20631,
 -20787, -20942, -21096, -21249, -21402, -21554, -21705, -21855,
 -22004, -22153, -22301, -22448, -22594, -22739, -22883, -23027,
 -23169, -23311, -23452, -23592, -23731, -23869, -24006, -24143,
 -24278, -24413, -24546, -24679, -24811, -24942, -25072, -25201,
 -25329, -25456, -25582, -25707, -25831, -25954, -26077, -26198,
 -26318, -26437, -26556, -26673, -26789, -26905, -27019, -27132,
 -27244, -27355, -27466, -27575, -27683, -27790, -27896, -28001,
 -28105, -28208, -28309, -28410, -28510, -28608, -28706, -28802,
 -28897, -28992, -29085, -29177, -29268, -29358, -29446, -29534,
 -29621, -29706, -29790, -29873, -29955, -30036, -30116, -30195,
 -30272, -30349, -30424, -30498, -30571, -30643, -30713, -30783,
 -30851, -30918, -30984, -31049, -31113, -31175, -31236, -31297,
 -31356, -31413, -31470, -31525, -31580, -31633, -31684, -31735,
 -31785, -31833, -31880, -31926, -31970, -32014, -32056, -32097,
 -32137, -32176, -32213, -32249, -32284, -32318, -32350, -32382,
 -32412, -32441, -32468, -32495, -32520, -32544, -32567, -32588,
 -32609, -32628, -32646, -32662, -32678, -32692, -32705, -32717,
 -32727, -32736, -32744, -32751, -32757, -32761, -32764, -32766,
};
#else
{};
#error "The selected points number N_WAVE in fft.h is not supported."
#endif

/*
  FIX_MPY() - fixed-point multiplication & scaling.
  Substitute inline assembly for hardware-specific
  optimization suited to a particluar DSP processor.
  Scaling ensures that result remains 16-bit.
*/
inline int16_t FIX_MPY(int16_t a, int16_t b)
{
	/* shift right one less bit (i.e. 15-1) */
	int32_t c = ((int32_t)a * (int32_t)b) >> 14;
	/* last bit shifted out = rounding-bit */
	b = c & 0x01;
	/* last shift + rounding bit */
	a = (c >> 1) + b;
	return a;
}

/*
  fix_fft() - perform forward/inverse fast Fourier transform.
  fr[n],fi[n] are real and imaginary arrays, both INPUT AND
  RESULT (in-place FFT), with 0 <= n < 2**m; set inverse to
  0 for forward transform (FFT), or 1 for iFFT.
*/
int16_t fix_fft(int16_t fr[], int16_t fi[], int16_t m, int16_t inverse)
{
	int16_t mr, nn, i, j, l, k, istep, n, scale, shift;
	int16_t qr, qi, tr, ti, wr, wi;

	n = 1 << m;

	/* max FFT size = N_WAVE */
	if (n > N_WAVE)
		return -1;

	mr = 0;
	nn = n - 1;
	scale = 0;

	/* decimation in time - re-order data */
	for (m=1; m<=nn; ++m) {
		l = n;
		do {
			l >>= 1;
		} while (mr+l > nn);
		mr = (mr & (l-1)) + l;

		if (mr <= m)
			continue;
		tr = fr[m];
		fr[m] = fr[mr];
		fr[mr] = tr;
		ti = fi[m];
		fi[m] = fi[mr];
		fi[mr] = ti;
	}

	l = 1;
	k = LOG2_N_WAVE-1;
	while (l < n) {
		if (inverse) {
			/* variable scaling, depending upon data */
			shift = 0;
			for (i=0; i<n; ++i) {
				j = fr[i];
				if (j < 0)
					j = -j;
				m = fi[i];
				if (m < 0)
					m = -m;
				if (j > 16383 || m > 16383) {
					shift = 1;
					break;
				}
			}
			if (shift)
				++scale;
		} else {
			/*
			  fixed scaling, for proper normalization --
			  there will be log2(n) passes, so this results
			  in an overall factor of 1/n, distributed to
			  maximize arithmetic accuracy.
			*/
			shift = 1;
		}
		/*
		  it may not be obvious, but the shift will be
		  performed on each data point exactly once,
		  during this pass.
		*/
		istep = l << 1;
		for (m=0; m<l; ++m) {
			j = m << k;
			/* 0 <= j < N_WAVE/2 */
			wr =  Sinewave[j+N_WAVE/4];
			wi = -Sinewave[j];
			if (inverse)
				wi = -wi;
			if (shift) {
				wr >>= 1;
				wi >>= 1;
			}
			for (i=m; i<n; i+=istep) {
				j = i + l;
				tr = FIX_MPY(wr,fr[j]) - FIX_MPY(wi,fi[j]);
				ti = FIX_MPY(wr,fi[j]) + FIX_MPY(wi,fr[j]);
				qr = fr[i];
				qi = fi[i];
				if (shift) {
					qr >>= 1;
					qi >>= 1;
				}
				fr[j] = qr - tr;
				fi[j] = qi - ti;
				fr[i] = qr + tr;
				fi[i] = qi + ti;
			}
		}
		--k;
		l = istep;
	}
	return scale;
}

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
/*
int16_t fix_fftr(int16_t f[], int16_t m, int16_t inverse)
{
	int i, N = 1<<(m-1), scale = 0;
	short tt, *fr=f, *fi=&f[N];

	if (inverse)
		scale = fix_fft(fi, fr, m-1, inverse);
	for (i=1; i<N; i+=2) {
		tt = f[N+i-1];
		f[N+i-1] = f[i];
		f[i] = tt;
	}
	if (! inverse)
		scale = fix_fft(fi, fr, m-1, inverse);
	return scale;
}

*/

uint16_t isqrt32(uint32_t n)  
    {  
        uint16_t c = 0x8000;  
        uint16_t g = 0x8000;  
      
        for(;;) {  
            if((uint32_t)g*g > n)  
                g ^= c;  
            c >>= 1;  
            if(c == 0)  
                return g;  
            g |= c;  
        }  
    } 


void fix_fft_cp2m(int16_t *vReal, int16_t *vImag, uint16_t samples)
{
/* vM is half the size of vReal and vImag */
	for (uint16_t i = 0; i < samples; i++) {
		vReal[i] = isqrt32( (int32_t)vReal[i]*vReal[i] + (int32_t)vImag[i]*vImag[i] );
	}
}


void  fix_fft_debias(int16_t *pdSamples, uint16_t n) {
    int32_t mean = 0;
    uint16_t i;
    int16_t m16;
    for(i=0; i<n; i++) {
        mean+=pdSamples[i];
    }
    m16=mean/n;
    for(i=0; i<n; i++) {
        pdSamples[i]-=m16;
    }
  }

void fix_fft_sq2(int16_t *pdSamples, uint16_t n) {    
    uint16_t i;
    for(i=0; i<(n>>1); i++) {
        pdSamples[i]=(pdSamples[i*2]+pdSamples[i*2+1])>>1;
    }
}

void fix_fft_denoise(int16_t *pdSamples, uint16_t n, int16_t th) {
    uint16_t i;
    for(i=0; i<n; i++) {
        if(pdSamples[i]<th) pdSamples[i]=0;
    }
}

//7 : X:2,5,7,12
//8 : L:12,18,18,24

void  fix_fft_log(int16_t *pdSamples, uint16_t n) {
    int16_t  value;
    uint16_t  result, i;
    for(i=0; i<n; i++) {
        value = pdSamples[i];
        if(value>1) {            
            //pdSamples[i]=20.0*log10(value);
            
            // log2
            // * 20 * log10(2) = 20 * 3/10 = 6
            uint16_t uv=value;
            result=0;
            while( uv>>=1 ) result++;

            uv=1<<result;
            //if((uint32_t)(value-uv)*10>=15) result++; //round
            //uv=(uint32_t)value*10/uv;
            //uv = uv<11 ? 0 : uv<13 ? 1 : uv<14 ? 2 : uv<16 ? 3 : uv<18 ? 4 : 5;
            uv=((uint32_t)value*10-(uint32_t)uv*10)/uv/2;
            pdSamples[i]=result*6+uv; 
            //pdSamples[i]=result*6;     
                  
        }
        else     
            pdSamples[i]=0;
    }
  }
   

void fix_fft_wnd(int16_t *vData, uint16_t samples, uint8_t windowType, uint8_t dir)
{
/* Weighing factors are computed once before multiple use of FFT */
/* The weighing function is symetric; half the weighs are recorded */
	double samplesMinusOne = (double(samples) - 1.0);
	for (uint16_t i = 0; i < (samples >> 1); i++) {
		double indexMinusOne = double(i);
		double ratio = (indexMinusOne / samplesMinusOne);
		double weighingFactor = 1.0;
		/* Compute and record weighting factor */
		switch (windowType) {
		case FFT_WIN_TYP_RECTANGLE: /* rectangle (box car) */
			weighingFactor = 1.0;
			break;
		case FFT_WIN_TYP_HAMMING: /* hamming */
			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_HANN: /* hann */
			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_TRIANGLE: /* triangle (Bartlett) */
			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
			break;
		case FFT_WIN_TYP_BLACKMAN: /* blackmann */
			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
			break;
		case FFT_WIN_TYP_FLT_TOP: /* flat top */
			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
			break;
		case FFT_WIN_TYP_WELCH: /* welch */
			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
			break;
		}
		if (dir == FFT_FORWARD) {
			vData[i] = weighingFactor*vData[i];
			vData[samples - (i + 1)] = weighingFactor*vData[samples - (i + 1)];
		}
		else {
			vData[i] /= weighingFactor;
			vData[samples - (i + 1)] /= weighingFactor;
		}
	}
}