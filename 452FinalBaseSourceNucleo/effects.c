//
//  effects.c
//  
//
//  Created by Timothy Kennedy on 06/11/18.
//
//

#include <stdio.h>
#include <stdlib.h>

//#include "effects.h"

// pwrap.c - pointer wrapping relative to circular buffer
// Usage: p_new = pwrap(D,w,p)
// movable pointer p shifted for circular buffer
/*
 * ith state Si and update calculated by:
 * Si = *pwrap(D,w,p+i) i = 1,2,...D
 * pnext = pwrap(D,w,--p)
 * example:
 * y = *pwrap(D,w,p+d); 	delay output
 * *p=x; 					delay-line input
 * p = pwrap(D,w,--p); 		backshift circular buffer pointer
 */
// -------------------------------------------------------//
//#define D 2500
//float w[D+1];
float a,b,c,s0;
float *p;

void initializeEffectParameters()
{
	a = 0.75;
	b = (1-a);
	c = 512;
}


float* pwrap(int D, float *w, float *p)
{
if (p > w+D)
p -= D+1;
if (p < w)
p += D+1;
return p;
}

//TODO: change reverb funcs to use local pwrap functions. pointers MUST match process, can't swap around


// reverb.c - reverb using circular delay buffer
void reverb(int16_t* inL, int16_t* outL, int16_t* inR, int16_t* outR){
int D = 2500;
float w[D+1];
	float* x;
	float y, sD;
	*x = (float) *inR;
	sD = *pwrap(D,&w,p+D); // extract D-th state relative to p
	y=*x + a*sD; // compute output sample
	*p = y; // delay-line input
	p = pwrap(D,&w,--p); // backshift pointer
	*outR = (int16_t) y;
}
/*
float reverbplain(int D, float *w, float **p, float a, float x)
{
float y, sD;
sD = *pwrap(D,w,*p+D);
y = x + a * sD;
**p = y;
*p = pwrap(D,w,--*p);
return y;
}
// reverballpass.c - allpass reverb with circular delay line - canonical realization
// ---------------------------------------------------------------------------

void reverballpass(int16_t* inL, int16_t* outL, int16_t* inR, int16_t* outR, int D, float *w, float **p, float x)
{
	float *x;
	float y;
	//float y, s0, sD;
	*x = (float) *inR;
	sD = *pwrap(D,w,*p+D);
	s0 = x + a * sD;
	y = -a * s0 + sD;
	**p = s0;
	*p = pwrap(D,w,--*p);
	*outR = (int16_t) y;
}
*/
int16_t thresh(int16_t* x)
{
	float y, xc = *x/c; // this y is local to f()
	y = *x * (1 - b * xc *xc);

	if(*x<c) y=a*c;
	if(*x<-c) y = -a*c;

	return (int16_t) y;
}
// soft.c - guitar distortion by soft thresholding // ---------------------------------------------------------------------------------
void distortion(int16_t* inL, int16_t* inR, int16_t* outL, int16_t* outR){
	*outL =  thresh(&inL);
		*outL = *outL << 1;
	*outR = thresh(&inR);
		*outR = *outR << 1;
}


//legacy
/*
void distortion(int16_t* inL, int16_t* inR, int16_t* outL, int16_t* outR){
	*outL =  (int16_t) thresh((int) &inL);
		*outL = *outL << 1;
	*outR = (int16_t) thresh((int) &inR);
		*outR = *outR << 1;
}
int thresh(int* x)
{
	float y, xc = *x/c; // this y is local to f()
	y = *x * (1 - b * xc *xc);

	if(*x<c) y=a*c;
	if(*x<-c) y = -a*c;

	return ((int) y);
}
*/


