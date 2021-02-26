#include "grbl.h"
#ifdef PLT_V2
  #include "MaslowDue.h"
  // Define different comment types for pre-parsing.
  #define COMMENT_NONE 0
#endif
#define bufferLength 64          // serial buffer length

File root;
Sd2Card card;



//----------------------------------------------------------------------
void init_SD()
{
    pinMode(SD_CS_PIN, OUTPUT);
    delay_us(1000);
    if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}

//----------------------------------------------------------------------

void getFileList()
{ //Serial.print out all the .txt files on the SD card
    root = SD.open("/");

  printDirectory(root, 0);

  Serial.println("done!");
}
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

//----------------------------------------------------------------------
void deleteFile(char *filename, int length)
{

    init_SD();
    root = SD.open("/");
    root.rewindDirectory();

    if (SD.remove(filename))
    {
        Serial.print("DELETED ");
        Serial.println(filename);
    }
    else
    {
        Serial.print("FAILED to delete ");
        Serial.println(filename);
    }
    if (root)
    {
        root.close();
    }
}

//----------------------------------------------------------------------
unsigned int getFilesize(char *filename, int length)
{

    return 0;
}

void drawFromSD(char *filename, int length)
{ // start the draw from SD
    // make sure the filename is null terminated:
    filename[length] = '\0';
    init_SD();

    Serial.print("print:  ");
    Serial.println(filename);
    File file = SD.open(filename);

    if (file)
    {
        Serial.print("Starting to parse ");
        Serial.println(file.name());

        //travelDistance = 0L;
        parseFileContents(0,file);

        Serial.println(" --- done parsing (disable steppers)");

       // Serial.println(MACHINE_STOPPED); // c0 = ended drawing, a command the controller software understands
        file.close();
        //disable_steppers();
    }
    else
    {
        Serial.print("I couldn't find the file ");
        Serial.println(filename);
    }
}

//----------------------------------------------------------------------
bool hasExtension(File file, char *ext)
{
    //we only support extensions that are 3 chars long (for now) : a dot '.' plus the name
    //first find length of filename
    int namelen = 0;

    while (true)
    {
        char c = file.name()[namelen];
        if (c == '\0')
            break;
        if (namelen > 13)
        { // should NEVER happen, is just a safety
            Serial.println("ERROR the filename is too long, should only be 13 chars long (see SD.h)");
            break;
        }
        namelen++;
    }

    if (namelen < 4)
        return false; // 3 chars and the "." before it = 4
    // now check it..;
    for (int i = 0; i < 4; i++)
    {
        if (ext[4 - 1 - i] != file.name()[namelen - 1 - i])
            return false;
    }
    return true;
}

//----------------------------------------------------------------------
void parseFileContents(long position,File file)
{

    char buf[bufferLength];
    int i = 0;

    unsigned int line = 0;
    bool ignore_chars = 0;
    int safe_move = 0;

    // remember where the pen was in the real world
    //LongPoint memPos;
    //  	if(bPreview){
    //  		memPos.x = current_pos.x;
    //  		memPos.y = current_pos.y;
    //  		memPos.z = current_pos.z;
    //  	}
    //	offSet.x = current_pos.x;
    //	offSet.y = current_pos.y;
    //
    char c;
    int status = 0;
 
    if (file)
    {
       
        file.seek(position);
        while (file.available())
        {
            while(status != 0){

            }
            c = file.read();
            if (i < bufferLength)
            {
                switch (c)
                {
                case '\n': // newline
                case '\r':
                    buf[i] = '\0'; // end string
                    
                    if (ignore_chars == 0)
                    {
                     // TODO to parser  //parseMessage(buf, i);
                     status = 1;
                    gc_execute_line(buf);
                    }
                    else
                    {
                        line = 0;
                    }
                    i = 0;
                    ignore_chars = 0;

                    break;

                case '#': //ignore comments
                    ignore_chars = 1;
                    break;
                default:
                    if (ignore_chars == 0)
                    {
                        buf[i] = c;
                        i++;
                        Serial.print(c);
                    }

                    break;
                }
            }
            else
            {
                Serial.println("ERROR buffer is full! (BAD READ)");
                i = 0;
                break;
            }
        }
        
        // lift pen
        //		moveTo(current_pos.x, current_pos.y, 100); // brush up at current position
        //		moveTo(offSet.x, offSet.y);
        //  		if(bPreview){
        //  			// set the pen back to where it was.
        //  			set_position(memPos.x, memPos.y, memPos.z);
        //  		}
        //		offSet.x = 0;
        //		offSet.y = 0;
    }
    else
    {
        Serial.println("parseFileContents got passed a bad file");
    }
}

int get_data_at_line(uint8_t linenumber,const char *dir,const char *filename){
    //init_SD();
    root = SD.open(dir);
int p = 0;
    if(root){
        printPgmString("ok root");
        File entry =  root.openNextFile();
        printPgmString(entry.name());
        printPgmString("  \n");
        p =get_position_line(1,entry);
        parseFileContents(p,entry);
        printPgmString("  ");
          p =get_position_line(2,entry);
        parseFileContents(p,entry);
        printPgmString("  ");
          p =get_position_line(3,entry);
        parseFileContents(p,entry);
        printPgmString("  ");
        entry.close();
        }
    else{
        printPgmString("false root");
    }

     root.rewindDirectory();
 
  root.close();

    
return 0;

}

void file_parser(char *line, uint8_t length){
   switch (line[0])
    {
    case 'P':
     
        drawFromSD(&line[1], length - 1);
        break;
   

    default: // bad command
        Serial.print("? bad command: ");
        Serial.println(line[0]);
        break;
    }
 
}

int get_position_line(int linenumber,File file){
    int pos = 0;
    int found = 0;
    int line_counter = 1;

    char c;
    if (file)
    {
        file.seek(0);
        while (file.available())
        {
            c = file.read();
           
            if (line_counter == linenumber){
                found =1;
                break;
            }
             else if(c =='\n'||c =='\r'){
                line_counter++;
                pos++;
                }
                
            else{
                pos++;
                }
        }
    }
    
    
    if(!found){
        pos = 0;
    }
  
    
    return pos;
}
