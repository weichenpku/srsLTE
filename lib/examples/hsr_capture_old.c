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
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <stdbool.h>

#include "srslte/srslte.h"
#include "srslte/phy/rf/rf.h"
#include "srslte/phy/rf/rf_utils.h"

#define MHZ             1000000
#define SAMP_FREQ       1920000
#define FLEN            9600
#define FLEN_PERIOD     0.005
#define MAX_EARFCN 1000
int band = -1;
int earfcn_start=-1, earfcn_end = -1;

cell_search_cfg_t cell_detect_config = {
  SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
  SRSLTE_DEFAULT_MAX_FRAMES_PSS, 
  SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
  0
};

struct cells {
  srslte_cell_t cell;
  float freq; 
  int dl_earfcn;
  float power;
};
struct cells results[1024]; 
uint32_t n_found_cells=0;

static bool keep_running = true;
char *output_file_name = NULL;
char *rf_args="";
float rf_gain=70.0, rf_freq=-1.0;
int nof_prb = 6;
int nof_subframes = -1;
int N_id_2 = -1; 
uint32_t nof_rx_antennas = 1;

void int_handler(int dummy) {
  keep_running = false;
}

void usage(char *prog) {
  printf("Usage: %s [agrnv] -l N_id_2 -f rx_frequency_hz -o output_file\n", prog);
  printf("\t-a RF args [Default %s]\n", rf_args);
  printf("\t-g RF Gain [Default %.2f dB]\n", rf_gain);
  printf("\t-p nof_prb [Default %d]\n", nof_prb);
  printf("\t-s earfcn_start [Default All]\n");
  printf("\t-e earfcn_end [Default All]\n");
  printf("\t-n nof_subframes [Default %d]\n", nof_subframes);
  printf("\t-A nof_rx_antennas [Default %d]\n", nof_rx_antennas);
  printf("\t-v verbose\n");
}

void parse_args(int argc, char **argv) {
  int opt;
  while ((opt = getopt(argc, argv, "agpnvfolAbse")) != -1) {
    switch (opt) {
    case 'o':
      output_file_name = argv[optind];
      break;
    case 'a':
      rf_args = argv[optind];
      break;
    case 'g':
      rf_gain = atof(argv[optind]);
      break;
    case 'p':
      nof_prb = atoi(argv[optind]);
      break;
    case 'f':
      rf_freq = atof(argv[optind]);
      break;
    case 'n':
      nof_subframes = atoi(argv[optind]);
      break;
    case 'l':
      N_id_2 = atoi(argv[optind]);
      break;
    case 'A':
      nof_rx_antennas = (uint32_t) atoi(argv[optind]);
      break;
    case 'v':
      srslte_verbose++;
      break;
    case 's':
      earfcn_start = atoi(argv[optind]);
      break;
    case 'e':
      earfcn_end = atoi(argv[optind]);
      break;
    case 'b':
      band = atoi(argv[optind]);
      break;
    default:
      usage(argv[0]);
      exit(-1);
    }
  }
}

int srslte_rf_recv_wrapper1(void *h, cf_t *data[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t *t) {
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  return srslte_rf_recv_with_time_multi(h, (void**) data, nsamples, true, NULL, NULL);
}

int srslte_rf_recv_wrapper(void *h, cf_t *data[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t *t) {
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  void *ptr[SRSLTE_MAX_PORTS];
  for (int i=0;i<SRSLTE_MAX_PORTS;i++) {
    ptr[i] = data[i];
  }
  return srslte_rf_recv_with_time_multi((srslte_rf_t*) h, ptr, nsamples, 1, NULL, NULL);
}

bool go_exit = false; 

void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

double srslte_rf_set_rx_gain_wrapper(void *h, double f) {
  return srslte_rf_set_rx_gain((srslte_rf_t*) h, f);
}

void cell_search(){
  int n; 
  srslte_rf_t rf;
  srslte_ue_cellsearch_t cs; 
  srslte_ue_cellsearch_result_t found_cells[3]; 
  int nof_freqs; 
  srslte_earfcn_t channels[MAX_EARFCN];
  uint32_t freq;
  n_found_cells=0;

    
  printf("Opening RF device...\n");
  if (srslte_rf_open(&rf, rf_args)) {
    fprintf(stderr, "Error opening rf\n");
    exit(-1);
  }  
  if (!cell_detect_config.init_agc) {
    srslte_rf_set_rx_gain(&rf, rf_gain);
  } else {
    printf("Starting AGC thread...\n");
    if (srslte_rf_start_gain_thread(&rf, false)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    srslte_rf_set_rx_gain(&rf, 50);      
  }

  srslte_rf_set_master_clock_rate(&rf, 30.72e6);        

  // Supress RF messages
  srslte_rf_suppress_stdout(&rf);
  
  nof_freqs = srslte_band_get_fd_band(band, channels, earfcn_start, earfcn_end, MAX_EARFCN);
  if (nof_freqs < 0) {
    fprintf(stderr, "Error getting EARFCN list\n");
    exit(-1);
  }

  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  signal(SIGINT, sig_int_handler);

  if (srslte_ue_cellsearch_init_multi(&cs, cell_detect_config.max_frames_pss, srslte_rf_recv_wrapper, 1, (void*) &rf)) {
    fprintf(stderr, "Error initiating UE cell detect\n");
    exit(-1);
  }

  if (cell_detect_config.max_frames_pss) {
    srslte_ue_cellsearch_set_nof_valid_frames(&cs, cell_detect_config.nof_valid_pss_frames);
  }
  if (cell_detect_config.init_agc) {
    srslte_ue_sync_start_agc(&cs.ue_sync, srslte_rf_set_rx_gain_wrapper, cell_detect_config.init_agc);
  }

  for (freq=0;freq<nof_freqs && !go_exit;freq++) {
  
    /* set rf_freq */
    srslte_rf_set_rx_freq(&rf, (double) channels[freq].fd * MHZ);
    srslte_rf_rx_wait_lo_locked(&rf);
    INFO("Set rf_freq to %.3f MHz\n", (double) channels[freq].fd * MHZ/1000000);
    
    printf("[%3d/%d]: EARFCN %d Freq. %.2f MHz looking for PSS.\n", freq, nof_freqs,
                      channels[freq].id, channels[freq].fd);fflush(stdout);
    
    if (SRSLTE_VERBOSE_ISINFO()) {
      printf("\n");
    }
      
    bzero(found_cells, 3*sizeof(srslte_ue_cellsearch_result_t));

    INFO("Setting sampling frequency %.2f MHz for PSS search\n", SRSLTE_CS_SAMP_FREQ/1000000);
    srslte_rf_set_rx_srate(&rf, SRSLTE_CS_SAMP_FREQ);
    INFO("Starting receiver...\n");
    srslte_rf_start_rx_stream(&rf, false);
    
    n = srslte_ue_cellsearch_scan(&cs, found_cells, NULL); 
    if (n < 0) {
      fprintf(stderr, "Error searching cell\n");
      exit(-1);
    } else if (n > 0) {
      for (int i=0;i<3;i++) {
        if (found_cells[i].psr > 10.0) {
          srslte_cell_t cell;
          cell.id = found_cells[i].cell_id; 
          cell.cp = found_cells[i].cp; 
          int ret = rf_mib_decoder(&rf, 1, &cell_detect_config, &cell, NULL);
          if (ret < 0) {
            fprintf(stderr, "Error decoding MIB\n");
            exit(-1);
          }
          if (ret == SRSLTE_UE_MIB_FOUND) {
            printf("Found CELL ID %d. %d PRB, %d ports\n", 
                 cell.id, 
                 cell.nof_prb, 
                 cell.nof_ports);
            if (cell.nof_ports > 0) {
              memcpy(&results[n_found_cells].cell, &cell, sizeof(srslte_cell_t));
              results[n_found_cells].freq = channels[freq].fd; 
              results[n_found_cells].dl_earfcn = channels[freq].id;
              results[n_found_cells].power = found_cells[i].peak;
              n_found_cells++;
            }
            go_exit=true;
            break;
          }          
        }
      }
    }    
  }

  printf("\n\nFound %d cells\n", n_found_cells);
  for (int i=0;i<n_found_cells;i++) {
    printf("Found CELL %.1f MHz, EARFCN=%d, PHYID=%d, %d PRB, %d ports, PSS power=%.1f dBm\n", 
           results[i].freq,
           results[i].dl_earfcn,
           results[i].cell.id, 
           results[i].cell.nof_prb, 
           results[i].cell.nof_ports, 
           10*log10(results[i].power));

  }
  srslte_ue_cellsearch_free(&cs);
  srslte_rf_close(&rf);
}

int main(int argc, char **argv) {
  FILE* fd;
  char * path="/media/soar/My Passport/data/7.5.2";
  parse_args(argc, argv);
  while(true){
    go_exit=false;
    cell_search();
    if (n_found_cells==0){
      printf("No cell found\n");
      break;
    }
    int maxcell = 0;
    float maxpower = results[0].power;
    for (int i=0;i<n_found_cells;i++){
      if (results[i].power>maxpower){
        maxpower = results[i].power;
        maxcell = i;
      }
    }

    int repeat_time = nof_subframes/1000;
    nof_subframes/=repeat_time;
    for (int k=0;k<repeat_time;k++){
      time_t cur_time;
      struct tm *t;
      time(&cur_time);
      t=gmtime(&cur_time);
      char time_buf[100];
      output_file_name = time_buf; 
      //sprintf(output_file_name,"data/%d_%d_%d:%d:%d_%d.bin",t->tm_mon+1,t->tm_mday,t->tm_hour+8,t->tm_min,t->tm_sec,k);
      sprintf(output_file_name,"%s/%d.%d.%d-%d-%d_%d.bin",path,t->tm_mon+1,t->tm_mday,t->tm_hour+8,t->tm_min,t->tm_sec,k);

      time_t start_time = time(NULL);
      rf_gain = 70.0;
      rf_freq = results[maxcell].freq*1000000;
      nof_prb = results[maxcell].cell.nof_prb;
      N_id_2 = results[maxcell].cell.id; 
      nof_rx_antennas = 2;
      printf("Capture cell: %f MHz, %d PRB, PYHID=%d into %s\n",rf_freq/1000000,nof_prb,N_id_2,output_file_name);
      cf_t *buffer[SRSLTE_MAX_PORTS] = {NULL, NULL}; 
      int n;
      srslte_rf_t rf;
      srslte_filesink_t sink;
      srslte_ue_sync_t ue_sync; 
      srslte_cell_t cell; 

      signal(SIGINT, int_handler);
      
      srslte_filesink_init(&sink, output_file_name, SRSLTE_COMPLEX_FLOAT_BIN);

      printf("Opening RF device...\n");
      if (srslte_rf_open_multi(&rf, rf_args, nof_rx_antennas)) {
        fprintf(stderr, "Error opening rf\n");
        exit(-1);
      }
      srslte_rf_set_master_clock_rate(&rf, 30.72e6);        

      for (int i = 0; i< SRSLTE_MAX_PORTS; i++) {
        buffer[i] = srslte_vec_malloc(3 * sizeof(cf_t) * SRSLTE_SF_LEN_PRB(100));
      }
      
      sigset_t sigset;
      sigemptyset(&sigset);
      sigaddset(&sigset, SIGINT);
      sigprocmask(SIG_UNBLOCK, &sigset, NULL);

      printf("Set RX freq: %.6f MHz\n", srslte_rf_set_rx_freq(&rf, rf_freq) / 1000000);
      printf("Set RX gain: %.1f dB\n", srslte_rf_set_rx_gain(&rf, rf_gain));
        int srate = srslte_sampling_freq_hz(nof_prb);    
        if (srate != -1) {  
          if (srate < 10e6) {          
            srslte_rf_set_master_clock_rate(&rf, 4*srate);        
          } else {
            srslte_rf_set_master_clock_rate(&rf, srate);        
          }
          printf("Setting sampling rate %.2f MHz\n", (float) srate/1000000);
          float srate_rf = srslte_rf_set_rx_srate(&rf, (double) srate);
          if (srate_rf != srate) {
            fprintf(stderr, "Could not set sampling rate\n");
            exit(-1);
          }
        } else {
          fprintf(stderr, "Invalid number of PRB %d\n", nof_prb);
          exit(-1);
        }
      srslte_rf_rx_wait_lo_locked(&rf);
      srslte_rf_start_rx_stream(&rf, false);

      cell.cp = SRSLTE_CP_NORM; 
      cell.id = N_id_2;
      cell.nof_prb = nof_prb; 
      cell.nof_ports = 1; 
      
      if (srslte_ue_sync_init_multi(&ue_sync, cell.nof_prb, cell.id==1000, srslte_rf_recv_wrapper1, nof_rx_antennas, (void*) &rf)) {
        fprintf(stderr, "Error initiating ue_sync\n");
        exit(-1); 
      }
      if (srslte_ue_sync_set_cell(&ue_sync, cell)) {
        fprintf(stderr, "Error initiating ue_sync\n");
        exit(-1);
      }

      uint32_t subframe_count = 0;
      bool start_capture = false; 
      bool stop_capture = false; 
      while((subframe_count < nof_subframes || nof_subframes == -1)
            && !stop_capture)
      {
        n = srslte_ue_sync_zerocopy_multi(&ue_sync, buffer);
        if (n < 0) {
          fprintf(stderr, "Error receiving samples\n");
          exit(-1);
        }
        if (n == 1) {
          if (!start_capture) {
            if (srslte_ue_sync_get_sfidx(&ue_sync) == 9) {
              start_capture = true; 
            }        
          } else {
            printf("Writing to file %6d subframes...\r", subframe_count);
            srslte_filesink_write_multi(&sink, (void**) buffer, SRSLTE_SF_LEN_PRB(nof_prb),nof_rx_antennas);
            subframe_count++;                              
          }
        }
        if (!keep_running) {
          if (!start_capture || (start_capture && srslte_ue_sync_get_sfidx(&ue_sync) == 9)) {
            stop_capture = true; 
          }
        }
      }

      time_t end_time = time(NULL);

      char datainfo[100];
      sprintf(datainfo, "%s/datainfo.csv", path);
      fd=fopen(datainfo,"a");
      fprintf(fd,"%s,%.1f,%d,%d,%d,%d,%.1f,%d,%f\n",
             output_file_name,
             results[maxcell].freq,
             results[maxcell].dl_earfcn,
             results[maxcell].cell.id, 
             results[maxcell].cell.nof_prb, 
             results[maxcell].cell.nof_ports, 
             10*log10(results[maxcell].power),
             nof_subframes,
             difftime(end_time,start_time));
      fclose(fd);

      srslte_filesink_free(&sink);
      srslte_rf_close(&rf);
      srslte_ue_sync_free(&ue_sync);

      for (int i = 0; i < SRSLTE_MAX_PORTS; i++) {
        if (buffer[i]) {
          free(buffer[i]);
        }
      }

      printf("\nOk - wrote %d subframes\n", subframe_count);
    }
  }
  exit(0);
}
