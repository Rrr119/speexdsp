/* Double-talk AEC test driver.

   Runs the synthetic microphone/far-end signals produced by
   testecho_dtd_gen through the echo canceller in two configurations:
   AEC alone, and AEC with the residual-echo-suppression preprocessor.

   It reports:
   - ERLE in far-end-only frames (echo attenuation in dB);
   - near-end speech preservation during the annotated double-talk windows,
     measured as the RMS energy ratio between the AEC output and the clean
     near-end reference (1.0 means the near-end speaker is fully preserved);
   - double-talk detector hit rate inside the windows and false-alarm rate
     outside them.
   The driver exits non-zero if any metric falls outside its tolerance. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define NN 128
#define TAIL 1024
#define RATE 8000

#define MIN_ERLE_DB 12.f
/* These are energy (RMS) preservation ratios in the annotated double-talk
   windows, which contain both near-end speech and imperfectly cancelled
   echo. The AEC-only floor matches the unmodified canceller's behaviour on
   this scenario; the AEC+RESS floor is set above the baseline post-filter
   result, so a regression that crushes the near end during double-talk fails. */
#define MIN_NEAR_GAIN_AEC 0.40f
#define MIN_NEAR_GAIN_FULL 0.30f
#define MIN_DT_HIT_RATE 0.7f
#define MAX_DT_FALSE_RATE 0.2f

typedef struct {
   int start, end;
} DtWindow;

static short *read_pcm(const char *path, int *count)
{
   FILE *f = fopen(path, "rb");
   long len;
   short *buf;
   if (!f)
   {
      fprintf(stderr, "cannot open %s\n", path);
      return NULL;
   }
   fseek(f, 0, SEEK_END);
   len = ftell(f) / (long)sizeof(short);
   fseek(f, 0, SEEK_SET);
   buf = (short*)malloc((size_t)len * sizeof(short));
   if (fread(buf, sizeof(short), (size_t)len, f) != (size_t)len)
   {
      fprintf(stderr, "short read on %s\n", path);
      free(buf);
      fclose(f);
      return NULL;
   }
   fclose(f);
   *count = (int)len;
   return buf;
}

static double frame_energy(const short *s, int n)
{
   double e = 0.0;
   int i;
   for (i = 0; i < n; i++)
      e += (double)s[i] * s[i];
   return e;
}

static int in_dt(int frame, const DtWindow *w, int nw)
{
   int i;
   for (i = 0; i < nw; i++)
      if (frame >= w[i].start && frame <= w[i].end)
         return 1;
   return 0;
}

int main(int argc, char **argv)
{
   const char *dir = argc > 1 ? argv[1] : ".";
   char path[1024];
   short *mic, *far, *near;
   short out_aec[NN], out_full[NN];
   int nmic, nfar, nnear, frames, f, i;
   int nw = 0;
   DtWindow windows[8];
   SpeexEchoState *aec_a, *aec_b;
   SpeexPreprocessState *pre;
   int rate = RATE;

   double echo_only_mic_e = 0.0, echo_only_aec_e = 0.0, echo_only_full_e = 0.0;
   double dt_near_e = 0.0, dt_aec_e = 0.0, dt_full_e = 0.0;
   int dt_hits = 0, dt_frames = 0, fa_frames = 0, fa_count = 0;
   double erle_db, gain_aec, gain_full;
   double hit_rate, false_rate;
   int failures = 0;

   FILE *cfg;

   snprintf(path, sizeof(path), "%s/mic.sw", dir);
   mic = read_pcm(path, &nmic);
   snprintf(path, sizeof(path), "%s/far.sw", dir);
   far = read_pcm(path, &nfar);
   snprintf(path, sizeof(path), "%s/near.sw", dir);
   near = read_pcm(path, &nnear);
   if (!mic || !far || !near)
      return 1;
   if (!(nmic == nfar && nfar == nnear))
   {
      fprintf(stderr, "input length mismatch: mic=%d far=%d near=%d\n", nmic, nfar, nnear);
      return 1;
   }

   snprintf(path, sizeof(path), "%s/config.txt", dir);
   cfg = fopen(path, "r");
   if (!cfg)
   {
      fprintf(stderr, "cannot open %s\n", path);
      return 1;
   }
   {
      char line[512];
      while (fgets(line, sizeof(line), cfg))
      {
         if (strncmp(line, "doubletalk=", 11) == 0)
         {
            char *p = line + 11;
            while (nw < 8 && sscanf(p, "%d,%d", &windows[nw].start, &windows[nw].end) == 2)
            {
               nw++;
               while (*p && *p != ' ')
                  p++;
               while (*p == ' ')
                  p++;
            }
         }
      }
   }
   fclose(cfg);
   if (nw == 0)
   {
      fprintf(stderr, "no double-talk windows found in %s\n", path);
      return 1;
   }

   frames = nmic / NN;
   aec_a = speex_echo_state_init(NN, TAIL);
   aec_b = speex_echo_state_init(NN, TAIL);
   pre = speex_preprocess_state_init(NN, rate);
   speex_echo_ctl(aec_a, SPEEX_ECHO_SET_SAMPLING_RATE, &rate);
   speex_echo_ctl(aec_b, SPEEX_ECHO_SET_SAMPLING_RATE, &rate);
   speex_preprocess_ctl(pre, SPEEX_PREPROCESS_SET_ECHO_STATE, aec_b);

   for (f = 0; f < frames; f++)
   {
      int base = f * NN;
      int is_dt = in_dt(f, windows, nw);
      int dt_flag;
      speex_echo_cancellation(aec_a, mic + base, far + base, out_aec);
      speex_echo_cancellation(aec_b, mic + base, far + base, out_full);
      speex_preprocess_run(pre, out_full);

      speex_echo_ctl(aec_a, SPEEX_ECHO_GET_DOUBLETALK, &dt_flag);
      if (is_dt)
      {
         double ref_e = frame_energy(near + base, NN);
         dt_near_e += ref_e;
         dt_aec_e += frame_energy(out_aec, NN);
         dt_full_e += frame_energy(out_full, NN);
         dt_frames++;
         if (dt_flag)
            dt_hits++;
      } else
      {
         double mic_e = frame_energy(mic + base, NN);
         /* Only score far-end-active frames so pauses do not inflate ERLE */
         if (mic_e > NN * 1.0e4)
         {
            echo_only_mic_e += mic_e;
            echo_only_aec_e += frame_energy(out_aec, NN);
            echo_only_full_e += frame_energy(out_full, NN);
            fa_frames++;
            if (dt_flag)
               fa_count++;
         }
      }
   }

   erle_db = 10.0 * log10((echo_only_mic_e + 1.0) / (echo_only_aec_e + 1.0));
   /* Near-end preservation: RMS ratio between the AEC output in the annotated
      double-talk windows and the clean near-end reference. A value near 1 means
      the near-end speaker survives; strong suppression drives it toward 0.
      Energy ratios are robust to the residual echo and phase differences that
      make a least-squares cross-gain unreliable here. */
   gain_aec = sqrt((dt_aec_e + 1.0) / (dt_near_e + 1.0));
   gain_full = sqrt((dt_full_e + 1.0) / (dt_near_e + 1.0));
   hit_rate = dt_frames ? (double)dt_hits / dt_frames : 0.0;
   false_rate = fa_frames ? (double)fa_count / fa_frames : 0.0;

   printf("AEC double-talk test:\n");
   printf("  ERLE (far-end only)          : %6.2f dB  (min %.1f)\n", erle_db, MIN_ERLE_DB);
   printf("  Near-end gain, AEC only      : %6.3f   (min %.2f)\n", gain_aec, MIN_NEAR_GAIN_AEC);
   printf("  Near-end gain, AEC+RESS      : %6.3f   (min %.2f)\n", gain_full, MIN_NEAR_GAIN_FULL);
   printf("  DTD hit rate in DT windows   : %6.3f   (min %.2f)\n", hit_rate, MIN_DT_HIT_RATE);
   printf("  DTD false alarm, far-only    : %6.3f   (max %.2f)\n", false_rate, MAX_DT_FALSE_RATE);

   if (erle_db < MIN_ERLE_DB) { fprintf(stderr, "FAIL: ERLE too low\n"); failures++; }
   if (gain_aec < MIN_NEAR_GAIN_AEC) { fprintf(stderr, "FAIL: near-end speech crushed by AEC\n"); failures++; }
   if (gain_full < MIN_NEAR_GAIN_FULL) { fprintf(stderr, "FAIL: near-end speech crushed by residual echo suppressor\n"); failures++; }
   if (hit_rate < MIN_DT_HIT_RATE) { fprintf(stderr, "FAIL: double-talk detector misses too many frames\n"); failures++; }
   if (false_rate > MAX_DT_FALSE_RATE) { fprintf(stderr, "FAIL: double-talk detector false alarms too often\n"); failures++; }

   speex_echo_state_destroy(aec_a);
   speex_echo_state_destroy(aec_b);
   speex_preprocess_state_destroy(pre);
   free(mic);
   free(far);
   free(near);
   if (failures)
   {
      fprintf(stderr, "testecho_dtd: %d metric(s) out of tolerance\n", failures);
      return 1;
   }
   printf("PASS\n");
   return 0;
}
