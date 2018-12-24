/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <stdbool.h>

#include "srslte/srslte.h"


#ifndef DISABLE_GRAPHICS
void init_plots();
void do_plots(float *corr, float energy, uint32_t size, cf_t ce[SRSLTE_PSS_LEN]);
void do_plots_sss(float *corr_m0, float *corr_m1);
void do_plot_cfo(float* cfo_list, int cfo_num);
void destroy_plots();
#endif


bool disable_plots = false;
char *input_file_name;
int cell_id = -1;
int nof_frames = -1;
uint32_t fft_size=64;
float threshold = 0.4; 
int N_id_2_sync = -1;
srslte_cp_t cp=SRSLTE_CP_NORM;
int file_offset = 0; 
bool use_standard_symbol_size = false;
float force_cfo = 2560.0/15000; // cfo caused by hardware is about +2543Hz
bool wait_for_char = false;

void usage(char *prog) {
  printf("Usage: %s [nlestodv] -i cell_id -f input_file_name\n", prog);
  printf("\t-n nof_frames [Default %d]\n", nof_frames);
  printf("\t-l N_id_2 to sync [Default use cell_id]\n");
  printf("\t-e Extended CP [Default Normal]\n");
  printf("\t-s symbol_sz [Default %d]\n", fft_size);
  printf("\t-t threshold [Default %.2f]\n", threshold);
  printf("\t-o file read offset [Default %d]\n", file_offset);
#ifndef DISABLE_GRAPHICS
  printf("\t-d disable plots [Default enabled]\n");
#else
  printf("\t plots are disabled. Graphics library not available\n");
#endif
  printf("\t-v srslte_verbose\n");
}

void parse_args(int argc, char **argv) {
  int opt;
  while ((opt = getopt(argc, argv, "nlestdvoifrw")) != -1) {
    switch (opt) {
    case 'f':
      input_file_name = argv[optind];
      break;
    case 't':
      threshold = atof(argv[optind]);
      break;
    case 'e':
      cp = SRSLTE_CP_EXT;
      break;
    case 'i':
      cell_id = atoi(argv[optind]);
      break;
    case 'o':
      file_offset = atoi(argv[optind]);
      break;
    case 'l':
      N_id_2_sync = atoi(argv[optind]);
      break;
    case 's':
      fft_size = atoi(argv[optind]);
      break;
    case 'n':
      nof_frames = atoi(argv[optind]);
      break;
    case 'd':
      disable_plots = true;
      break;
    case 'v':
      srslte_verbose++;
      break;
    case 'r':
      use_standard_symbol_size = true;
      break;
    case 'w':
      wait_for_char = true;
      break;
    default:
      usage(argv[0]);
      exit(-1);
    }
  }
  if (cell_id < 0) {
    usage(argv[0]);
    exit(-1);
  }
}
float m0_value, m1_value; 

int main(int argc, char **argv) {
  srslte_filesource_t fsrc;
  cf_t *buffer;
  int frame_cnt, n;
  srslte_pss_t pss;
  srslte_cfo_t cfocorr, cfocorr64; 
  srslte_sss_t sss;
  int32_t flen; 
  int peak_idx, last_peak;
  float peak_value; 
  float mean_peak; 
  uint32_t nof_det, nof_nodet, nof_nopeak, nof_nopeakdet;
  cf_t ce[SRSLTE_PSS_LEN]; 

  float* cfo_list;
  int cfo_num = 0;
  
  if (use_standard_symbol_size){
    srslte_use_standard_symbol_size(true);
  }
  
  parse_args(argc, argv);

  if (N_id_2_sync == -1) {
    N_id_2_sync = cell_id%3;
  }
  uint32_t N_id_2 = cell_id%3;
  uint32_t N_id_1 = cell_id/3;

#ifndef DISABLE_GRAPHICS
  if (!disable_plots)
    init_plots();
#endif

  cfo_list = malloc(sizeof(float) * nof_frames);

  flen = fft_size*15*5;

  buffer = malloc(sizeof(cf_t) * flen * 2);
  if (!buffer) {
    perror("malloc");
    exit(-1);
  }
    
  if (srslte_pss_init_fft(&pss, flen, fft_size)) {
    fprintf(stderr, "Error initiating PSS\n");
    exit(-1);
  }

  if (srslte_pss_set_N_id_2(&pss, N_id_2_sync)) {
    fprintf(stderr, "Error setting N_id_2=%d\n",N_id_2_sync);
    exit(-1);
  }
  
  srslte_cfo_init(&cfocorr, flen); 
  srslte_cfo_init(&cfocorr64, flen); 
 
  if (srslte_sss_init(&sss, fft_size)) {
    fprintf(stderr, "Error initializing SSS object\n");
    return SRSLTE_ERROR;
  }

  srslte_sss_set_N_id_2(&sss, N_id_2);

  printf("Opening file...\n");
  if (srslte_filesource_init(&fsrc, input_file_name, SRSLTE_COMPLEX_FLOAT_BIN)) {
    fprintf(stderr, "Error opening file %s\n", input_file_name);
    exit(-1);
  }

  printf("PSS detection threshold: %.2f\n", threshold);
  
  nof_det = nof_nodet = nof_nopeak = nof_nopeakdet = 0;
  frame_cnt = 0;
  last_peak = 0; 
  mean_peak = 0;
  int peak_offset = 0;
  float cfo = 0; 
  float mean_cfo = 0; 
  float mean_abs_cfo = 0;
  uint32_t m0, m1; 
  uint32_t sss_error1 = 0, sss_error2 = 0, sss_error3 = 0; 
  uint32_t cp_is_norm = 0; 
  
  srslte_sync_t ssync; 
  bzero(&ssync, sizeof(srslte_sync_t));
  ssync.fft_size = fft_size;
  
  n = srslte_filesource_read(&fsrc, buffer, file_offset);
  
  while(frame_cnt < nof_frames || nof_frames == -1) {
    /*
    n = srslte_filesource_read(&fsrc, buffer, 2*(flen - peak_offset));
    for (int i=0;i<flen - peak_offset;i++){
      buffer[i]=buffer[2*i];
    }
    */
    n = srslte_filesource_read(&fsrc, buffer, flen - peak_offset);
    if (n < 0) {
      fprintf(stderr, "Error reading samples\n");
      exit(-1);
    }
    if (n < flen - peak_offset) {
      fprintf(stdout, "End of file (n=%d, flen=%d, peak=%d)\n", n, flen, peak_offset);
      break;
    }
    
    peak_idx = srslte_pss_find_pss(&pss, buffer, &peak_value);
    if (peak_idx < 0) {
      fprintf(stderr, "Error finding PSS peak\n");
      exit(-1);
    }
    INFO("peak value is %f at %d from %d\n",peak_value, peak_idx, flen - peak_offset);
        
    mean_peak = SRSLTE_VEC_CMA(peak_value, mean_peak, frame_cnt);
    
    int N_id_1_partial=0, N_id_1_diff=0, N_id_1_full=0;
    if (peak_value >= threshold) {
      nof_det++;
        
      if (peak_idx >= fft_size) {

        // Estimate CFO 
        // Estimate CFO = Hardware CFO(~+2543 Hz) + Estimation error of CFO + Doppler shift
        cfo = srslte_pss_cfo_compute(&pss, &buffer[peak_idx-fft_size]);
        mean_cfo = SRSLTE_VEC_CMA(cfo, mean_cfo, frame_cnt);      
        mean_abs_cfo = SRSLTE_VEC_CMA(fabs(cfo - force_cfo), mean_abs_cfo, frame_cnt);

        // Correct CFO
        srslte_cfo_correct(&cfocorr, buffer, buffer, -mean_cfo / fft_size);               

        // Estimate channel
        if (srslte_pss_chest(&pss, &buffer[peak_idx-fft_size], ce)) {
          fprintf(stderr, "Error computing channel estimation\n");
          exit(-1);
        }
        /*
        for (int t=0;t<SRSLTE_PSS_LEN;t++){
          printf("(%d:%f+j*%f)  ",t,(__real__ ce[t]),(__imag__ ce[t]));
        }
        */
        
        // Find SSS 
        int sss_idx = peak_idx-2*fft_size-(SRSLTE_CP_ISNORM(cp)?SRSLTE_CP_LEN(fft_size, SRSLTE_CP_NORM_LEN):SRSLTE_CP_LEN(fft_size, SRSLTE_CP_EXT_LEN));             
        if (sss_idx >= 0 && sss_idx < flen-fft_size) {
          
          srslte_sss_m0m1_partial(&sss, &buffer[sss_idx], 3, NULL, &m0, &m0_value, &m1, &m1_value);
          N_id_1_partial =  srslte_sss_N_id_1(&sss, m0, m1);
          if (N_id_1_partial != N_id_1) {
            sss_error2++;            
          }
          INFO("sf_idx = %d\n", srslte_sss_subframe(m0, m1));
          INFO("Partial N_id_1: %d; m0: %d; m1: %d\n", N_id_1_partial,m0,m1);
          
          srslte_sss_m0m1_diff(&sss, &buffer[sss_idx], &m0, &m0_value, &m1, &m1_value);
          N_id_1_diff = srslte_sss_N_id_1(&sss, m0, m1);
          if (N_id_1_diff != N_id_1) {
            sss_error3++;            
          }
          INFO("Diff N_id_1: %d; m0: %d; m1: %d\n", N_id_1_diff,m0,m1);

          srslte_sss_m0m1_partial(&sss, &buffer[sss_idx], 1, NULL, &m0, &m0_value, &m1, &m1_value);
          N_id_1_full = srslte_sss_N_id_1(&sss, m0, m1);
          if (N_id_1_full != N_id_1) {
            sss_error1++;     
          }
          INFO("Full N_id_1: %d; m0: %d; m1: %d\n", N_id_1_full,m0,m1);
        }
        
        // Estimate CP 
        if (peak_idx > 2*(fft_size + SRSLTE_CP_LEN_EXT(fft_size))) {
          srslte_cp_t cp = srslte_sync_detect_cp(&ssync, buffer, peak_idx);
          if (SRSLTE_CP_ISNORM(cp)) {
            cp_is_norm++; 
          }       
        }
        
      } else {
        INFO("No space for CFO computation. Frame starts at \n");
      }
      
      if(srslte_sss_subframe(m0,m1) == 0)
      {
#ifndef DISABLE_GRAPHICS
          if (!disable_plots)
            do_plots_sss(sss.corr_output_m0, sss.corr_output_m1);
#endif
      }
      
    } else {
      nof_nodet++;
    }

    if (frame_cnt > 100) {
      if (abs(last_peak-peak_idx) > 4) {
        if (peak_value >= threshold) {
          nof_nopeakdet++;
        } 
        nof_nopeak++;                  
      } 
    }

    if (peak_value >= threshold && N_id_1_partial == N_id_1 && N_id_1_diff == N_id_1 && N_id_1_full == N_id_1){
      cfo_list[cfo_num] = (cfo - force_cfo)*15*1000;
      cfo_num++;
      /*
      FILE* fd;
      char* cfoinfo = "data/7.19.1/cfo10.csv";
      fd=fopen(cfoinfo,"a");
      fprintf(fd,"%d,%f\n",cfo_num-1,cfo_list[cfo_num-1]);
      fclose(fd);
      */
    }

    frame_cnt++;
    
    if (peak_value>threshold){
      printf("[%5d]: Pos: %5d, PSR: %4.1f (~%4.1f) Pdet: %4.2f, "
           "FA: %4.2f, CFO: %+4f Hz (~%4f Hz), Abs_CFO: %+4f Hz (~%4f Hz), SSSmiss: %4.2f/%4.2f/%4.2f CPNorm: %.0f%%\r", 
           frame_cnt, 
           peak_idx - flen/10, 
           peak_value, mean_peak,
           (float) nof_det/frame_cnt, 
           (float) nof_nopeakdet/frame_cnt, (cfo - force_cfo)*15*1000, (mean_cfo - force_cfo)*15*1000, fabs(cfo - force_cfo)*15*1000, mean_abs_cfo*15*1000,
           (float) sss_error1/nof_det,(float) sss_error2/nof_det,(float) sss_error3/nof_det,
           (float) cp_is_norm/nof_det * 100);
    }

    if (SRSLTE_VERBOSE_ISINFO()) {
      printf("\n");
    }

  
#ifndef DISABLE_GRAPHICS
    if (!disable_plots)
      do_plots(pss.conv_output_avg, pss.conv_output_avg[peak_idx], pss.fft_size+pss.frame_size-1, ce);
#endif
    if (wait_for_char){
      getchar();
    }
    last_peak = peak_idx;
  }

  srslte_pss_free(&pss);
  free(buffer);
  srslte_filesource_free(&fsrc);

  do_plot_cfo(cfo_list,cfo_num);
  getchar();

#ifndef DISABLE_GRAPHICS
  if (!disable_plots)
    destroy_plots();
#endif

  printf("Ok\n");
  exit(0);  
}

extern cf_t *tmp2;


/**********************************************************************
 *  Plotting Functions
 ***********************************************************************/
#ifndef DISABLE_GRAPHICS


#include "srsgui/srsgui.h"
plot_real_t pssout;
plot_complex_t pce; 

plot_real_t psss1, psss2;

plot_real_t pcfo;

float tmp[1000000];
cf_t tmpce[SRSLTE_PSS_LEN];


void init_plots() {
  sdrgui_init();
  plot_real_init(&pssout);
  plot_real_setTitle(&pssout, "PSS xCorr");
  plot_real_setLabels(&pssout, "Index", "Absolute value");
  plot_real_setYAxisScale(&pssout, 0, 1);
  
  
  plot_complex_init(&pce);
  plot_complex_setTitle(&pce, "Channel Estimates");
  plot_complex_setYAxisScale(&pce, Ip, -2, 2);
  plot_complex_setYAxisScale(&pce, Q, -2, 2);
  plot_complex_setYAxisScale(&pce, Magnitude, 0, 2);
  plot_complex_setYAxisScale(&pce, Phase, -M_PI, M_PI);
  
  
  plot_real_init(&psss1);
  plot_real_setTitle(&psss1, "SSS xCorr m0");
  plot_real_setLabels(&psss1, "Index", "Absolute value");
  plot_real_setYAxisScale(&psss1, 0, 1);
  
  
  plot_real_init(&psss2);
  plot_real_setTitle(&psss2, "SSS xCorr m1");
  plot_real_setLabels(&psss2, "Index", "Absolute value");
  plot_real_setYAxisScale(&psss2, 0, 1);
  
  plot_real_init(&pcfo);
  plot_real_setTitle(&pcfo, "CFO of each 5 pss");
  plot_real_setLabels(&pcfo, "Index of pss", "CFO value(Hz)");
  plot_real_setYAxisScale(&pcfo,-1000,1000);

}

void do_plots(float *corr, float energy, uint32_t size, cf_t ce[SRSLTE_PSS_LEN]) {  
  srslte_vec_sc_prod_fff(corr,1./energy,tmp, size);
  plot_real_setNewData(&pssout, tmp, size);        
  
  float norm = srslte_vec_avg_power_cf(ce, SRSLTE_PSS_LEN);
  srslte_vec_sc_prod_cfc(ce, 1.0/sqrt(norm), tmpce, SRSLTE_PSS_LEN);
  plot_complex_setNewData(&pce, tmpce, SRSLTE_PSS_LEN);
}

void do_plots_sss(float *corr_m0, float *corr_m1) {  
  if (m0_value > 0) 
    srslte_vec_sc_prod_fff(corr_m0,1./m0_value,corr_m0, SRSLTE_SSS_N);
  if (m1_value > 0) 
    srslte_vec_sc_prod_fff(corr_m1,1./m1_value,corr_m1, SRSLTE_SSS_N);
  plot_real_setNewData(&psss1, corr_m0, SRSLTE_SSS_N);
  plot_real_setNewData(&psss2, corr_m1, SRSLTE_SSS_N);        
}

void do_plot_cfo(float* cfo_list, int cfo_num){
  plot_real_setNewData(&pcfo, cfo_list, cfo_num);
}

void destroy_plots() {
  sdrgui_exit();
}


#endif
