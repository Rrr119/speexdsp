/* Generates synthetic acoustic echo cancellation test signals.

   The signals are deterministic and need no external audio material:
   - far.sw:   far-end speech (speaker), shaped multi-tone "speech-like"
               signal with talk/silence gaps;
   - near.sw:  near-end speech, active only in the double-talk windows;
   - mic.sw:   microphone signal = convolutive echo of far.sw through a
               simulated room impulse response + near.sw + low-level noise;
   - config.txt: frame-aligned scenario description, including the double-talk
               windows used by the AEC test driver.

   All files are raw signed 16-bit PCM, mono, 8 kHz. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RATE 8000
#define FRAME 128
#define TAIL 1024
#define DURATION 8
#define TOTAL (RATE*DURATION)

/* Double-talk windows in frames (see config.txt emitted alongside). */
#define DT1_START 200
#define DT1_END   299
#define DT2_START 380
#define DT2_END   449

static unsigned int rng_state = 0x12345678;
static float lp_state = 0.f;

static float frand(void)
{
   /* xorshift32, mapped to (-1, 1) */
   unsigned int x = rng_state;
   x ^= x << 13;
   x ^= x >> 17;
   x ^= x << 5;
   rng_state = x;
   return ((x >> 8) / (float)(1u << 24)) * 2.f - 1.f;
}

/* "Speech-like" source: colored (one-pole low-passed) noise mixed with a weak
   periodic component, gated by a slow talk envelope. The broadband excitation
   gives the adaptive filter a persistently rich spectrum to converge on,
   while the short talk gaps keep the signal non-stationary. The filter state
   is reset per source via the seed-dependent phase offset. */
static float speech_sample(int n, int seed)
{
   float t = n / (float)RATE;
   float f0 = 120.f + 20.f * sinf(0.5f * t + seed);
   float env;
   float phase = fmodf(t + seed * 1.37f, 1.8f);
   float white, tone;
   if (phase < 0.08f)
      env = phase / 0.08f;
   else if (phase < 1.6f)
      env = 1.f;
   else if (phase < 1.7f)
      env = (1.7f - phase) / 0.1f;
   else
      env = 0.f;
   white = frand();
   lp_state = 0.85f * lp_state + 0.15f * white;
   tone = 0.35f * sinf(2.f * M_PI * f0 * t + seed) +
          0.2f * sinf(2.f * M_PI * 2.f * f0 * t);
   return 0.55f * env * (0.7f * lp_state + tone);
}

static int near_active(int frame)
{
   return (frame >= DT1_START && frame <= DT1_END) ||
          (frame >= DT2_START && frame <= DT2_END);
}

int main(int argc, char **argv)
{
   const char *dir = argc > 1 ? argv[1] : ".";
   char path[1024];
   FILE *ff, *fn, *fm, *fc;
   float *far, *near, *echo;
   float ir[TAIL];
   short *out;
   int n, k, frames;

   far = (float*)calloc(TOTAL, sizeof(float));
   near = (float*)calloc(TOTAL, sizeof(float));
   echo = (float*)calloc(TOTAL, sizeof(float));
   out = (short*)malloc(TOTAL * sizeof(short));
   if (!far || !near || !echo || !out)
   {
      fprintf(stderr, "out of memory\n");
      return 1;
   }

   for (n = 0; n < TOTAL; n++)
      far[n] = speech_sample(n, 1);

   for (n = 0; n < TOTAL; n++)
   {
      if (near_active(n / FRAME))
         near[n] = 1.5f * speech_sample(n, 7);
   }

   /* Simulated room impulse response: a bulk delay followed by an exponential
      decay with a few distinct reflection peaks. */
   for (k = 0; k < TAIL; k++)
      ir[k] = 0.f;
   ir[40] = 0.7f;
   ir[95] = 0.32f;
   ir[180] = 0.2f;
   ir[300] = 0.12f;
   ir[520] = 0.07f;
   for (k = 40; k < TAIL; k++)
      ir[k] += 0.15f * expf(-(k - 40) / 240.f);

   for (n = 0; n < TOTAL; n++)
   {
      int lo = n - (TAIL - 1);
      int jmin = lo < 0 ? -lo : 0;
      float acc = 0.f;
      int k2;
      for (k2 = jmin; k2 < TAIL && lo + k2 < TOTAL; k2++)
         acc += ir[k2] * far[lo + k2];
      echo[n] = acc;
   }

   frames = TOTAL / FRAME;

   snprintf(path, sizeof(path), "%s/far.sw", dir);
   ff = fopen(path, "wb");
   snprintf(path, sizeof(path), "%s/near.sw", dir);
   fn = fopen(path, "wb");
   snprintf(path, sizeof(path), "%s/mic.sw", dir);
   fm = fopen(path, "wb");
   snprintf(path, sizeof(path), "%s/config.txt", dir);
   fc = fopen(path, "wb");
   if (!ff || !fn || !fm || !fc)
   {
      fprintf(stderr, "cannot open output files in %s\n", dir);
      return 1;
   }

   for (n = 0; n < TOTAL; n++)
      out[n] = (short)floor(far[n] * 20000.f + 0.5f);
   fwrite(out, sizeof(short), TOTAL, ff);

   for (n = 0; n < TOTAL; n++)
      out[n] = (short)floor(near[n] * 20000.f + 0.5f);
   fwrite(out, sizeof(short), TOTAL, fn);

   for (n = 0; n < TOTAL; n++)
   {
      float mic = echo[n] + near[n] + 0.004f * frand();
      if (mic > 1.f) mic = 1.f;
      if (mic < -1.f) mic = -1.f;
      out[n] = (short)floor(mic * 20000.f + 0.5f);
   }
   fwrite(out, sizeof(short), TOTAL, fm);

   fprintf(fc, "rate=%d\n", RATE);
   fprintf(fc, "frame_size=%d\n", FRAME);
   fprintf(fc, "tail=%d\n", TAIL);
   fprintf(fc, "frames=%d\n", frames);
   fprintf(fc, "doubletalk=%d,%d %d,%d\n", DT1_START, DT1_END, DT2_START, DT2_END);

   fclose(ff);
   fclose(fn);
   fclose(fm);
   fclose(fc);
   free(far);
   free(near);
   free(echo);
   free(out);
   printf("generated far.sw, near.sw, mic.sw and config.txt (%d frames) in %s\n",
          frames, dir);
   return 0;
}
