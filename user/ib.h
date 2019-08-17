#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define KERNEL_BUFFER_SIZE 0x228

int sendIR(unsigned leadingPulseWidth, unsigned leadingGapWidth,
           unsigned onePulseWidth, unsigned oneGapWidth,
           unsigned zeroPulseWidth, unsigned zeroGapWidth,
           unsigned trailingPulseWidth, unsigned frequency, unsigned dc_n,
           unsigned dc_m, unsigned char* data) {
  int fd, ret;
  char message[KERNEL_BUFFER_SIZE];

  memcpy(message, &leadingPulseWidth, sizeof leadingPulseWidth);
  memcpy(message + sizeof(int), &leadingGapWidth, sizeof leadingGapWidth);
  memcpy(message + (sizeof(int) * 2), &onePulseWidth, sizeof onePulseWidth);
  memcpy(message + (sizeof(int) * 3), &oneGapWidth, sizeof oneGapWidth);
  memcpy(message + (sizeof(int) * 4), &zeroPulseWidth, sizeof zeroPulseWidth);
  memcpy(message + (sizeof(int) * 5), &zeroGapWidth, sizeof zeroGapWidth);
  memcpy(message + (sizeof(int) * 6), &trailingPulseWidth,
         sizeof trailingPulseWidth);
  memcpy(message + (sizeof(int) * 7), &frequency, sizeof frequency);
  memcpy(message + (sizeof(int) * 8), &dc_n, sizeof dc_n);
  memcpy(message + (sizeof(int) * 9), &dc_m, sizeof dc_m);
  memcpy(message + (sizeof(int) * 10), data, 0x200);

  fd = open("/dev/irblaster", O_RDWR);
  if (fd < 0) {
    perror("Failed to open the device...");
    return errno;
  }
  printf("Writing message to the device with data: [%s].\n", data);
  ret = write(fd, message, KERNEL_BUFFER_SIZE);
  if (ret < 0) {
    perror("Failed to write the message to the device.");
    return errno;
  }
  ret = close(fd);
  if (ret < 0) {
    perror("Failed to close the device.");
    return errno;
  }
}
