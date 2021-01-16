

#ifndef filesys_h
#define filesys_h
#include "grbl.h"
#ifdef PLT_V2
  #include "MaslowDue.h"
  #include <SPI.h>
#include <SD.h>

  // Define different comment types for pre-parsing.
  #define COMMENT_NONE 0
#endif

#define bufferLength 64          // serial buffer length

void init_SD();
void getFileList();
void deleteFile(char *filename, int length);
unsigned int getFilesize(char *filename, int length);
bool hasExtension(File file, char *ext);
void parseFileContents(File file);



#endif
