#include "grbl.h"
#ifdef PLT_V2
  #include "MaslowDue.h"
  // Define different comment types for pre-parsing.
  #define COMMENT_NONE 0
#endif

File root;

//----------------------------------------------------------------------
void init_SD()
{
    pinMode(CSPIN, OUTPUT);
    if (!SD.begin(CSPIN))
    {
        Serial.println(" SD initialization failed!");
        return;
    }
}

//----------------------------------------------------------------------

void getFileList()
{ //Serial.print out all the .txt files on the SD card
    init_SD();
    root = SD.open("/");
    Serial.print(" SD root: ");
    Serial.println(root);
    root.rewindDirectory();

    Serial.println("Files:");
    while (true)
    {
        File entry = root.openNextFile();

        if (!entry)
        {

            break;
        }
        if (!entry.isDirectory())
        {
            // do some more checking
            char *filename = entry.name();

            if (filename[0] == '~' || filename[0] == '.' || filename[0] == '_')
            {
                entry.close();
                continue;
            }
            if (hasExtension(entry, ".txt"))
            {
                entry.close();
                continue;
            }
            // if we are here, we have a good file, so Serial.print it
            Serial.print("filename:  ");
            Serial.println(entry.name());
            delayMicroseconds(5);
        }
        entry.close();
    }
    if (root)
    {
        root.close();
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
        parseFileContents(file);

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
void parseFileContents(File file)
{

    Serial.print("\tParsing file: ");
    Serial.println(file.name());
    Serial.println();
    char buf[bufferLength];
    int i = 0;
    unsigned int max_line_counter = 0;
    unsigned int line_counter = 0;
    int start_line = 0;

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
    if (file)
    {
        while (file.available())
        {
            c = file.read();
            if (c == '\n')
                max_line_counter++;
        }
         file.seek(0);
         while (line_counter < start_line) //get one line bevor to move there and make a line from there
            {
                c = file.read();
                if (c == '\n')
                    line_counter++;
            }
        printf("  lines: %d  ", line_counter);
    }
    else
    {
        printf("no lines counted\n");
    }
   
    if (file)
    {
       
        start_line > 0? safe_move = 1:safe_move = 0;
        while (file.available())
        {

            //start at a individual line
            // 

           
            // printf("start printing\n");
            // check serial to see if I have to stop
            
            //TODO abbruchbedingung
            // if (Serial.available() > 0)
            // {
            //     Serial.println("\n Drawing stopped by user");
            //     //terminate_drawing();
            //     break; // break out of while loop?
            // }
            c = file.read();
            if (i < bufferLength)
            {
                switch (c)
                {
                case '\n': // newline
                case '\r':
                    buf[i] = '\0'; // end string
                    printf("line %d / %d \n", ++line_counter, max_line_counter);
                    if (ignore_chars == 0)
                    {
                     // TODO to parser  //parseMessage(buf, i);
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

