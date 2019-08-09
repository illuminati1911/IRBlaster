#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>
 
#define BUFFER_LENGTH 1024               ///< The buffer length (crude but fine)
static char receive[BUFFER_LENGTH];     ///< The receive buffer from the LKM

struct Config {
   unsigned leadingPulseWidth;
   unsigned leadingGapWidth;
   unsigned onePulseWidth;
   unsigned oneGapWidth;
   unsigned zeroPulseWidth;
   unsigned zeroGapWidth;
   unsigned trailingPulseWidth;
   unsigned char code[0x200];
} *cfg;

char message[0x21C];

int main(){
   char *buffer;
   unsigned lpw = 9000;
   unsigned lgw = 4500;
   unsigned opw = 560;
   unsigned ogw = 1680;
   unsigned zpw = 560;
   unsigned zgw = 560;
   unsigned utp = 560;
   char *binary = "11001101001100100000111111110000";
   memcpy(message, &lpw, sizeof lpw);
   memcpy(message + sizeof(int), &lgw, sizeof lgw);
   memcpy(message + (sizeof(int) * 2), &opw, sizeof opw);
   memcpy(message + (sizeof(int) * 3), &ogw, sizeof ogw);
   memcpy(message + (sizeof(int) * 4), &zpw, sizeof zpw);
   memcpy(message + (sizeof(int) * 5), &zgw, sizeof zgw);
   memcpy(message + (sizeof(int) * 6), &utp, sizeof utp);
   memcpy(message + (sizeof(int) * 7), binary, 0x200);
   /*cfg = (struct Config *)&message;
   printf("Value: %d\n", cfg->useTrailingPulse);
   printf("Value: %s\n", cfg->code);*/
   int ret, fd;
   //char stringToSend[BUFFER_LENGTH] = {0};
   printf("Starting device test code example...\n");
   fd = open("/dev/irblaster", O_RDWR);             // Open the device with read/write access
   if (fd < 0){
      perror("Failed to open the device...");
      return errno;
   }
   //printf("Type in a short string to send to the kernel module:\n");
   //scanf("%[^\n]%*c", stringToSend);                // Read in a string (with spaces)
   //stringToSend[1] = strtol("00000000", 0, 2);
   printf("Writing message to the device [%s].\n", message);
   ret = write(fd, message, 0x21C); // Send the string to the LKM
   if (ret < 0){
      perror("Failed to write the message to the device.");
      return errno;
   }
   /*printf("Press ENTER to read back from the device...\n");
   getchar();
 
   printf("Reading from the device...\n");
   ret = read(fd, receive, BUFFER_LENGTH);        // Read the response from the LKM
   if (ret < 0){
      perror("Failed to read the message from the device.");
      return errno;
   }
   printf("The received message is: [%s]\n", receive);
   printf("End of the program\n");*/
   return 0;
}