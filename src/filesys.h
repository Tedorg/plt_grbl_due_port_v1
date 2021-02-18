

#ifndef filesys_h
#define filesys_h

#ifdef PLT_V2
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
void parseFileContents(long position,File file);
int get_data_at_line(uint8_t linenumber,const char *dir,const char *filename);
void file_parser(char *line, uint8_t length);
int get_position_line(int linenumber,File file);

#endif
