// zf240525.1033

// File system FS esp32-c3
#include <FS.h>          //this needs to be first
#include <LittleFS.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#define CFGFILE "/config.json"

struct iData{
  char name[33] = "Densimètre num 1";
  uint32_t sleeptime = 15 * 60;
  int16_t Offset[6];    //axOffsetInternal, ayOffsetInternal, azOffsetInternal, axOffset, ayOffset, azOffset
};

iData myData;


// LittleFS
bool formatLittleFS()
{
  USBSerial.print("\nneed to format LittleFS: ");
  LittleFS.end();
  LittleFS.begin();
  USBSerial.println(LittleFS.format());
  return LittleFS.begin();
}


bool mountFS(){
#ifdef DEBUG
  USBSerial.print("mounting fs...\n");
#endif
  // if LittleFS is not usable
  if (!LittleFS.begin()){
    USBSerial.println("Failed to mount file system");
    if (!formatLittleFS()){
      USBSerial.println("Failed to format file system - hardware issues!");
      return false;
    }
  }
  return true;
}


bool saveConfig(){
#ifdef DEBUG
  USBSerial.println("saving config...");
#endif
  JsonDocument doc;
  doc["Name"] = myData.name;
  doc["Sleep"] = myData.sleeptime;
  JsonArray array = doc["Offset"].to<JsonArray>();
  for (auto &&i : myData.Offset)
  {
    array.add(i);
  }
  mountFS();
  File configFile = LittleFS.open(CFGFILE, "w");
  if (!configFile)
  {
    USBSerial.println("failed to open config file for writing");
    LittleFS.end();
    return false;
  }
  else
  {
    serializeJson(doc, configFile);
#ifdef DEBUG
    USBSerial.println("serializeJson...");
    serializeJson(doc, USBSerial);
#endif
    configFile.flush();
    configFile.close();
#ifdef DEBUG
    USBSerial.println("\nsaved successfully");
#endif
    return true;
  }
}


bool readConfig(){
#ifdef DEBUG
  USBSerial.println("read config...");
  USBSerial.print("mounting FS...");
#endif
  if (!LittleFS.begin())
  {
    USBSerial.println(" ERROR: failed to mount FS!");
    return false;
  }
  else
  {
#ifdef DEBUG
    USBSerial.println(" mounted!");
#endif
    if (!LittleFS.exists(CFGFILE))
    {
      USBSerial.println("ERROR: failed to load json config");
      return false;
    }
    else
    {
      // file exists, reading and loading
#ifdef DEBUG
      USBSerial.println("reading config file");
#endif
      File configFile = LittleFS.open(CFGFILE, "r");
      if (!configFile)
      {
        USBSerial.println("ERROR: unable to open config file");
      }
      else
      {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, configFile);
        if (error)
        {
          USBSerial.print("deserializeJson() failed: ");
          USBSerial.println(error.c_str());
        }
        else
        {
          if (doc.containsKey("Name"))
            strcpy(myData.name, doc["Name"]);
          if (doc.containsKey("Sleep"))
            myData.sleeptime = doc["Sleep"];
          if (doc.containsKey("Offset"))
          {
            for (size_t i = 0; i < (sizeof(myData.Offset) / sizeof(*myData.Offset)); i++)
            {
              myData.Offset[i] = doc["Offset"][i];
            }
          }
#ifdef DEBUG
          USBSerial.println("parsed config:");
          serializeJson(doc, USBSerial);
          USBSerial.println("");
#endif
        }
      }
    }
  }
  return true;
}


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    USBSerial.printf("Listing directory: %s\r\n", dirname);
    File root = fs.open(dirname);
    if(!root){
        USBSerial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        USBSerial.println(" - not a directory");
        return;
    }
    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            USBSerial.print("  DIR : ");
            USBSerial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            USBSerial.print("  FILE: ");
            USBSerial.print(file.name());
            USBSerial.print("\tSIZE: ");
            USBSerial.println(file.size());
        }
        file = root.openNextFile();
    }
}


void readFile(fs::FS &fs, const char * path){
#ifdef DEBUG
    USBSerial.printf("Reading file: %s\r\n", path);
#endif
    File file = fs.open(path);
    if(!file || file.isDirectory()){
        USBSerial.println("- failed to open file for reading");
        return;
    }
#ifdef DEBUG
    USBSerial.println("- read from file:");
#endif
    while(file.available()){
        USBSerial.write(file.read());
    }
    USBSerial.println("");
    file.close();
}
