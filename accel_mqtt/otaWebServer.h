//
// OTA WEB server
//
// zf240514.1213
//
// Sources:
// https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
//

#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>


/*
 * Login page
 */
#include "loginIndex.h"


/*
 * Server Index Page
 */
#include "serverIndex.h"


WebServer server(80);

static void otaWebServer() {
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    USBSerial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  USBSerial.println("mDNS responder started");

  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      USBSerial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(USBSerial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(USBSerial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        USBSerial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(USBSerial);
      }
    }
  });
  server.begin();
}
