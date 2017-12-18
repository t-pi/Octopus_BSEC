/*
 * Changelog
 * 2017-12 t-pi: Added ESPWifi and ThingSpeak Interface from Octopus documentation / sketch GassensorThingspeak.ino
 * 2017-12 t-pi: Added Neopixel signaling: Left Pixel red if accuracy = 0, Right Pixel green / red between AQI 0..150
 */
/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved. 
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_iot_example.ino
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include "bsec_integration.h"
#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <Wire.h>

/**********************************************************************************************************************/
/* defines */
/**********************************************************************************************************************/

// Import access keys from separate file, s. example file
// needed are:
// WIFI
// #define MYSSID    <your Wifi's SSID>
// #define MYPWD     <your Wifi's passwort>
// ThingSpeak API key
// #define ThingSpeakAPI <your ThingSpeak API write key>
#include "access_keys.h"

// Which pin on the Arduino is connected to the NeoPixels?
// How many NeoPixels are attached to the Arduino?
// Octopus --> 13, Count 2
#define NEOPIN      13
#define NEOPIXELS   2

#define NEORIGHT    0
#define NEOLEFT     1

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel neopixels = Adafruit_NeoPixel(NEOPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

#define LEDPIN      2

// Aggregation --> average of # measurements
// Reduce amount of messages to ThingSpeak
#define AVERAGE 10
int count = 0;
float TStemp = 0;
float TShum = 0;
float TSpress = 0;
float TSiaq = 0;
float TSvoc = 0;
int TSaccuracy = 1;

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/


/*!
 * @brief           Write data to ThingSpeak Site
 *
 * param[in]        host      website to write to
 * param[in]        cmd       command to write
 * param[out]       antwort   reply from site
 *
 * @return          boolean, if ok
 * 
 * From original Octopus documentation sketch GassensorThingspeak.ino
 */
int httpGET(String host, String cmd, String &antwort) {
  WiFiClient client; // Client via ESP WiFi
  String text = "GET https://"+ host + cmd + " HTTP/1.1\r\n";
  text = text + "Host:" + host + "\r\n";
  text = text + "Connection:close\r\n\r\n";
  int ok = 1;
  if (ok) { // Connection available 
    ok = client.connect(host.c_str(),80);   // connecto to Client
    if (ok) {
      client.print(text);                   // send to Client 
      for (int tout=1000;tout>0 && client.available()==0; tout--)
        delay(10);                          // and wait for reply
      if (client.available() > 0)           // got reply 
        while (client.available())          // reading
          antwort = antwort + client.readStringUntil('\r');
      else ok = 0;
      client.stop(); 
      Serial.print(antwort);
    } 
  } 
  if (!ok) Serial.print(" no Wifi connection"); // Error
  return ok;
}

/*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */
 
    /* Write the data */
    for (int index = 0; index < data_len; index++) {
        Wire.write(reg_data_ptr[index]);
    }
 
    return (int8_t)Wire.endTransmission();
}

/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                    /* Set register address to start reading from */
    comResult = Wire.endTransmission();
 
    delayMicroseconds(150);                 /* Precautionary response delay */
    Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */
 
    int index = 0;
    while (Wire.available())  /* The slave device may send less than requested (burst read) */
    {
        reg_data_ptr[index] = Wire.read();
        index++;
    }
 
    return comResult;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void sleep(uint32_t t_ms)
{
    delay(t_ms);
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 * 
 * Changelog:
 * 2017-12 t-pi: Added Pressure and Raw VOC value output
 * 2017-12 t-pi: Added Neopixel signaling of accuracy / AQI
 * 2017-12 t-pi: Added ThingSpeak output of values
 * 
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status)
{
    Serial.print("[");
    Serial.print(timestamp/1e6);
    Serial.print("] T: ");
    Serial.print(temperature);
    Serial.print("| rH: ");
    Serial.print(humidity);
    Serial.print("| P: ");
    Serial.print(pressure);
    Serial.print("| IAQ: ");
    Serial.print(iaq);
    Serial.print(" (");
    Serial.print(iaq_accuracy);
    Serial.print(")");
    Serial.print("| VOC_ohm: ");
    Serial.println(gas);

    // Set Neopixel color according to AQI
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    if (iaq_accuracy==0) { // left pixel glows red, if accuracy == 0
        neopixels.setPixelColor(NEOLEFT, neopixels.Color(20,0,0)); 
    }
    else
    {   neopixels.setPixelColor(NEOLEFT, neopixels.Color(0,0,0));  
    }
    // right pixel red increases from 0..150, while green decreases
    // brightness limited to 30 to avoid sleepless nights
    // higher AQI are limited to 150
    neopixels.setPixelColor(NEORIGHT, neopixels.Color(iaq>150?30:iaq/5,iaq<150?30-(iaq/5):0,0));
    neopixels.show(); // This sends the updated pixel color to the hardware.

    // small blink of ESP-LED to indicate activity
    digitalWrite(LEDPIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);

    // calculate average of AVERAGE samples before uploading to ThingSpeak
    count++;
    if (count<=AVERAGE) { // sum up over AVERAGE readings
      TStemp += temperature;
      TShum += humidity;
      TSpress += pressure;
      TSiaq += iaq;
      TSvoc += gas; 
      // Accuracy is set to latest accuracy, only 0 is sticky for AVERAGE samples
      // --> used to detect any accuracy 0
      TSaccuracy = TSaccuracy==0 ? 0 : iaq_accuracy;    
    }
    else { // and divide the sum by AVERAGE
      TStemp /= AVERAGE;
      TShum /= AVERAGE;
      TSpress /= AVERAGE;
      TSiaq /= AVERAGE;
      TSvoc /= AVERAGE;      
      Serial.print("Sending data to ThingSpeak.... Msg #");
      // Send data to ThingSpeak site
      // 1: T, 2: rH, 3: P, 4: IAQ, 5: VOC, 6: Accuracy
      String cmd = "/update?api_key="+ String(ThingSpeakAPI);
      String host = "api.thingspeak.com";
      String antwort= " ";
      cmd = cmd +String("&field1="+String(TStemp)
        +"&field2="+String(TShum)
        +"&field3="+String(TSpress)
        +"&field4="+String(TSiaq)
        +"&field5="+String(TSvoc)
        +"&field6="+String(TSaccuracy))+ "\n\r";
      httpGET(host,cmd,antwort);// und absenden 
      Serial.println(" ok!");
      // Reset averaging
      count = 0;
      TStemp = 0;
      TShum = 0;
      TSpress = 0;
      TSiaq = 0;
      TSvoc = 0;
      TSaccuracy = 1;
    }
    digitalWrite(LED_BUILTIN, LOW);    // turn the ESP LED off after message sent
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
    return 0;
}

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
void setup()
{
    return_values_init ret;

    /* Init I2C and serial communication */
    Wire.begin();
    Serial.begin(115200);
    neopixels.begin(); // This initializes the NeoPixel library.
    pinMode(LEDPIN, OUTPUT);
  
    //------------ WLAN initialisieren 
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    delay(100);
    Serial.print ("\nWLAN connect to:");
    Serial.print(MYSSID);
    WiFi.begin(MYSSID,MYPWD);
    while (WiFi.status() != WL_CONNECTED) { // Warte bis Verbindung steht 
      delay(500); 
      Serial.print(".");
    };
    Serial.println ("\nconnected, my IP:"+ WiFi.localIP().toString());

   /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 5.0f, bus_write, bus_read, sleep, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        Serial.println("Error while initializing BME680");
        return;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        Serial.println("Error while initializing BSEC library");
        return;
    }

    // First playing with BME680 configuration - so far (2017-12-18) no success
    uint8_t serialized_settings[BSEC_MAX_PROPERTY_BLOB_SIZE];
    uint32_t n_serialized_settings_max = BSEC_MAX_PROPERTY_BLOB_SIZE;
    uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
    uint32_t n_work_buffer = BSEC_MAX_PROPERTY_BLOB_SIZE;
    uint32_t n_serialized_settings = 0;
    // Configuration of BSEC algorithm is stored in ’serialized_settings’
    ret.bsec_status = bsec_get_configuration(0, serialized_settings, n_serialized_settings_max, work_buffer, n_work_buffer, &n_serialized_settings);
    if (ret.bsec_status == BSEC_OK)
    {
        Serial.print("Settings length: ");
        Serial.println(n_serialized_settings);
        // ret.bsec_status = bsec_set_configuration(bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer));     
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        Serial.println("Error while setting configuration");
    }

    
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
}

void loop()
{
}

/*! @}*/

