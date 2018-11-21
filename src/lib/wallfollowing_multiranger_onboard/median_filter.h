/*
 * Copyright (c) 2012 Ted Carancho. (AeroQuad)
 * (c) 2012 Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#define MEDIAN_H

#define MAX_MEDIAN_DATASIZE 13
#define MEDIAN_DEFAULT_SIZE 5

//#include "std.h"
struct MedianFilterInt {
  int32_t data[MAX_MEDIAN_DATASIZE], sortData[MAX_MEDIAN_DATASIZE];
  uint8_t dataIndex;
  uint8_t size;
};

static inline void init_median_filter_i(struct MedianFilterInt *filter, uint8_t size)
{
  uint8_t i;
  if (size > MAX_MEDIAN_DATASIZE){
    filter->size = MAX_MEDIAN_DATASIZE;
  } else if ((size % 2) == 0) {
    // force filter to have odd number of entries so that
    // returned median is always an entry and not an average
    filter->size = size + 1;
  } else {
    filter->size = size;
  }
  for (i = 0; i < filter->size; i++) {
    filter->data[i] = 150;
    filter->sortData[i] = 150;
  }
  filter->dataIndex = 0;
}

static inline int32_t get_median_filter_i(struct MedianFilterInt *filter)
{
  if (filter->size % 2){
    return filter->sortData[filter->size >> 1];
  } else {
    // this should not be used if init_median_filter was used
    return (filter->sortData[filter->size / 2] + filter->sortData[filter->size / 2 - 1]) / 2;
  }
}

static inline int32_t update_median_filter_i(struct MedianFilterInt *filter, int32_t new_data)
{
  int temp, i, j; // used to sort array

  // Insert new data into raw data array round robin style
  filter->data[filter->dataIndex] = new_data;
  filter->dataIndex = (filter->dataIndex + 1) % filter->size;

  // Copy raw data to sort data array
  memcpy(filter->sortData, filter->data, sizeof(int32_t) * filter->size);

  // Insertion Sort
  for (i = 1; i < filter->size; i++) {
    temp = filter->sortData[i];
    j = i - 1;
    while (j >= 0 && temp < filter->sortData[j]) {
      filter->sortData[j + 1] = filter->sortData[j];
      j = j - 1;
    }
    filter->sortData[j + 1] = temp;
  }
  // return data value in middle of sorted array
  return get_median_filter_i(filter);
}

struct MedianFilterFloat {
  float data[MAX_MEDIAN_DATASIZE], sortData[MAX_MEDIAN_DATASIZE];
  uint8_t dataIndex;
  uint8_t size;
};

static inline void init_median_filter_f(struct MedianFilterFloat *filter, uint8_t size)
{
  uint8_t i;
  if (size > MAX_MEDIAN_DATASIZE){
    filter->size = MAX_MEDIAN_DATASIZE;
  } else if ((size % 2) == 0){
    filter->size = size + 1;
  } else {
    filter->size = size;
  }
  for (i = 0; i < filter->size; i++) {
    filter->data[i] = 0.f;
    filter->sortData[i] = 0.f;
  }
  filter->dataIndex = 0;
}

static inline float get_median_filter_f(struct MedianFilterFloat *filter)
{
  if (filter->size % 2){
    return filter->sortData[filter->size >> 1];
  } else {
    // this should not be used if init_median_filter was used
    return (filter->sortData[filter->size / 2] + filter->sortData[filter->size / 2 - 1]) / 2;
  }
}

static inline float update_median_filter_f(struct MedianFilterFloat *filter, float new_data)
{
  float temp;
  int i, j; // used to sort array

  // Insert new data into raw data array round robin style
  filter->data[filter->dataIndex] = new_data;
  filter->dataIndex = (filter->dataIndex + 1) % filter->size;

  // Copy raw data to sort data array
  memcpy(filter->sortData, filter->data, sizeof(float) * filter->size);

  // Insertion Sort
  for (i = 1; i < filter->size; i++) {
    temp = filter->sortData[i];
    j = i - 1;
    while (j >= 0 && temp < filter->sortData[j]) {
      filter->sortData[j + 1] = filter->sortData[j];
      j = j - 1;
    }
    filter->sortData[j + 1] = temp;
  }
  // return data value in middle of sorted array
  return get_median_filter_f(filter);
}

