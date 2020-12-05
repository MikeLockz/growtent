// This #include statement was automatically added by the Particle IDE.
#include <clickButton.h>

// This #include statement was automatically added by the Particle IDE.
#include <MQTT.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_BME680.h>

// This #include statement was automatically added by the Particle IDE.
// #include "Ubidots.h"

// This #include statement was automatically added by the Particle IDE.
#include <I2cMaster.h>

// This #include statement was automatically added by the Particle IDE.
#include "stdint.h"

// This #include statement was automatically added by the Particle IDE.
// #include <SparkFunBME280.h>

// This #include statement was automatically added by the Particle IDE.
#include <SparkFunMicroOLED.h>
#include "math.h"

/****************************************
 * Define Constants
 ****************************************/

// #ifndef TOKEN
// #define TOKEN "BBFF-kCrf1zugUVo40MJKdd6kR6uibPfbmG"  // Put here your Ubidots TOKEN
// #endif
#define VAR_ID "5c5b75141d8472235888d301" // Put here your data source name
#define VAR_LABEL "temp"                  // Put here your variable api label
#define DEVICE_LABEL "grow-tent"          // Put here your device api label

#define CCS811_ADDR 0x5B //Default I2C Address

#define SEALEVELPRESSURE_HPA (1013.25) // Sea level pressure for BME680

//long fiveMinutes = 5 * 60 * 1000UL;
long fiveMinutes = 1 * 30 * 1000UL;

/****************************************
 * Initialize Global Objects
 ****************************************/

// Ubidots ubidots(TOKEN);
MicroOLED oled;
//BME280 myBME280;
Adafruit_BME680 myBME680;
CCS811 myCCS811(CCS811_ADDR);

byte server[] = {192, 168, 0, 24};
MQTT client(server, 1883, callback);

/****************************************
 * Auxiliar Functions
 ****************************************/
class SoilSensor
{
    // Class member variables
    // Initialized at startup
    int soilPin;   //Declare a variable for the soil moisture sensor
    int soilPower; //Variable for Soil moisture Power
    long waitTime; //milliseconds of loop

    // For maintaining current state
    int calibratedMoistureDry;
    int calibratedMoistureWet;
    int lastSoilSensorReading;
    double calibratedPercentMoist;

    unsigned long previousMillis; //will store last time soil sensor was updated

    // Constructor - creates a soil sensor and initializes the member variables and state
public:
    SoilSensor(int pin, int power, long wait)
    {
        soilPin = pin;
        soilPower = power;

        waitTime = wait;

        calibratedMoistureDry = 0;
        calibratedMoistureWet = 3840;
        lastSoilSensorReading = 0;
        calibratedPercentMoist = 0.0;

        previousMillis = 0;
    }

    void Init()
    {
        pinMode(soilPower, OUTPUT);   //Set D7 as an OUTPUT
        digitalWrite(soilPower, LOW); //Set to LOW so no power is flowing through the sensor
    }

    void Update()
    {
        //check to see if it's time to change the state of the sensor
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= waitTime)
        {
            AverageSoilSensorValue();

            // remember the time
            previousMillis = currentMillis;
        }
    }

    // should return AIR or xx.x% moist
    double CalculateSoilMoisture()
    {
        Serial.println("Getting soil moisture");

        Serial.print("lastSoilSensorReading: ");
        Serial.print(lastSoilSensorReading);
        Serial.println();

        Serial.print("calibratedMoistureDry: ");
        Serial.print(calibratedMoistureDry);
        Serial.println();

        Serial.print("calibratedMoistureWet: ");
        Serial.print(calibratedMoistureWet);
        Serial.println();

        // Calculate percent moisture
        int numerator = lastSoilSensorReading - calibratedMoistureDry;
        Serial.print("numerator: ");
        Serial.print(numerator);
        Serial.println();

        double denominator = calibratedMoistureWet - calibratedMoistureDry;
        Serial.print("denominator: ");
        Serial.print(denominator);
        Serial.println();

        calibratedPercentMoist = numerator / denominator;

        Serial.print("calibratedPercentMoist: ");
        Serial.print(calibratedPercentMoist);
        Serial.println();

        calibratedPercentMoist = calibratedPercentMoist * 100;

        Serial.print("calibratedPercentMoist * 100: ");
        Serial.print(calibratedPercentMoist);
        Serial.println();

        // ubidots.add("Soil_Moisture", lastSoilSensorReading);
        // ubidots.add("Percent_Moist", calibratedPercentMoist);

        //publish the sensor output
        Particle.publish("soilMoistureReading", String(lastSoilSensorReading));
        Particle.publish("soilMoisturePercent", String(calibratedPercentMoist));

        return calibratedPercentMoist;
    };

    void CalibrateDry()
    {
        calibratedMoistureDry = AverageSoilSensorValue();
        CalculateSoilMoisture();
        Particle.publish("calibrateSoilMoistureDry", String(calibratedMoistureDry));
    }

    void CalibrateWet()
    {
        calibratedMoistureWet = AverageSoilSensorValue();
        CalculateSoilMoisture();
        Particle.publish("calibrateSoilMoistureWet", String(calibratedMoistureWet));
    }

    int AverageSoilSensorValue()
    {
        int sensorAverage = 0;

        // averageSensor = analogRead(soilPin);
        // turn on the sensor
        digitalWrite(soilPower, HIGH);
        delay(10);

        int totalReadings = 10;
        // read the sensor x10 and store values in array
        for (int i = 0; i <= totalReadings; i++)
        {
            Serial.print("Reading ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(analogRead(soilPin));
            Serial.println();
            sensorAverage += analogRead(soilPin);
            Serial.print("sensorAverage=");
            Serial.print(sensorAverage);
            Serial.println();
        }

        // turn off the sensor
        digitalWrite(soilPower, LOW);

        // take average/smooth array of readings
        Serial.print("sensorAverage ");
        Serial.print(sensorAverage);
        Serial.print(" / totalReadings ");
        Serial.print(totalReadings);
        Serial.println();
        sensorAverage = (sensorAverage / totalReadings);
        Serial.print("sensorAverage=");
        Serial.print(sensorAverage);
        Serial.println();
        lastSoilSensorReading = sensorAverage;

        CalculateSoilMoisture();

        return sensorAverage;
    };

    int getLastSoilSensorReading()
    {
        return lastSoilSensorReading;
    };

    double getPercentMoist()
    {
        return calibratedPercentMoist;
    }
};

SoilSensor soilSensor1(A0, 4, fiveMinutes);

// Instantiate instance of button w/ library
ClickButton button1(A1, LOW, CLICKBTN_PULLUP);
class GrowTentButton
{
    int buttonInterrupt;
    int buttonLed;

public:
    GrowTentButton(int initButtonInterrupt, int initButtonLed)
    {
        buttonInterrupt = initButtonInterrupt;
        buttonLed = initButtonLed;
    }

    void Init()
    {
        pinMode(buttonInterrupt, INPUT_PULLUP);
        pinMode(buttonLed, OUTPUT);
        pulseButton(1, 25);

        button1.debounceTime = 20;    // Debounce timer in ms
        button1.multiclickTime = 250; // Time limit for multi clicks
        button1.longClickTime = 1000;
    }
    void Update()
    {
        button1.Update();
        // Always run these
        evaluateButtonState(button1.clicks);
        delay(5);
    }

    void evaluateButtonState(int function)
    {
        if (function == 1)
        {
            Serial.println("SINGLE click - toggle on/off");
        };

        if (function == 2)
        {
            Serial.println("DOUBLE click - CALIBRATE DRY");
            soilSensor1.CalibrateDry();
            blinkButton(2, 200);
        };

        if (function == 3)
        {
            Serial.println("TRIPLE click - CALIBRATE WET");
            soilSensor1.CalibrateWet();
            blinkButton(3, 150);
        }

        if (function == -1)
        {
            Serial.println("SINGLE LONG click");
            soilSensor1.AverageSoilSensorValue();
            pulseButton(1, 20);
        }

        if (function == -2)
        {
            Serial.println("DOUBLE LONG click");
            pulseButton(2, 15);
        }

        if (function == -3)
        {
            Serial.println("TRIPLE LONG click");
            pulseButton(3, 10);
        }
    }

    void pulseButton(int repeats, int time)
    {
        for (int i = 0; i < repeats; i++)
        {
            //Fade LED on
            for (int fadeValue = 0; fadeValue <= 255; fadeValue += 5)
            {
                analogWrite(buttonLed, fadeValue);
                delay(time);
            }

            //Fade LED off
            for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 5)
            {
                analogWrite(buttonLed, fadeValue);
                delay(time);
            }
        }
    }

    void blinkButton(int repeats, int time)
    {
        for (int i = 0; i < repeats; i++)
        {
            digitalWrite(buttonLed, HIGH);
            delay(time);
            digitalWrite(buttonLed, LOW);
            delay(time);
        }
    }

    int getButtonInterrupt()
    {
        return buttonInterrupt;
    };
};
GrowTentButton growTentButton1(A1, 2);

class GrowTent
{
    unsigned long previousGrowMillis;
    int waitTimeGrow;
    unsigned long previousPublishMillis;
    int waitTimePublish;

public:
    GrowTent(int nothing)
    {
        previousGrowMillis = 0;
        waitTimeGrow = 1000;
        previousPublishMillis = 0;
        waitTimePublish = 60000;
    }

    void Init()
    {
        Serial.begin(115200);

        // ubidots.setMethod(TYPE_UDP);
        // ubidots.setDebug(true);

        // Connect to MQTT server
        // check for connection, reconnect if needed?
        client.connect("sparkclient");
        if (client.isConnected())
            Serial.println("Connected to mqtt.");

        // Config OLED
        oled.begin();
        oled.clear(ALL);
        oled.display();
        delay(1000);
        oled.clear(PAGE);

        //This begins the CCS811 sensor and prints error status of .begin()
        CCS811Core::status returnCode = myCCS811.begin();
        if (returnCode != CCS811Core::SENSOR_SUCCESS)
        {
            Serial.println("Problem with CCS811");
            printDriverError(returnCode);
        }
        else
        {
            Serial.println("CCS811 online");
        }

        //Initialize BME680
        BME680Init();

        // SWITCHED TO BME680
        //Initialize BME280
        //For I2C, enable the following and disable the SPI section
        // myBME280.settings.commInterface = I2C_MODE;
        // myBME280.settings.I2CAddress = 0x77;
        // myBME280.settings.runMode = 3; //Normal mode
        // myBME280.settings.tStandby = 0;
        // myBME280.settings.filter = 4;
        // myBME280.settings.tempOverSample = 5;
        // myBME280.settings.pressOverSample = 5;
        // myBME280.settings.humidOverSample = 5;

        // BME280Init();

        //Calling .begin() causes the settings to be loaded
        // delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
        // byte id = myBME280.begin(); //Returns ID of 0x60 if successful
        // if (id != 0x60)
        // {
        //   Serial.println("Problem with BME280");
        // }
        // else
        // {
        //   Serial.println("BME280 online");
        // }
    }

    void Update()
    {
        // Class properties
        unsigned long currentGrowMillis = millis();

        // Only run these periodically (1 sec)
        if (currentGrowMillis - previousGrowMillis >= waitTimeGrow)
        {
            //   updateCSS811();
            publishData(); // Update values via web services
            updateOled();  // Update OLED
            printSerial(); // Outputs readings to serial

            if (client.isConnected())
                client.loop();

            previousGrowMillis = currentGrowMillis;
        }
    }

    void BME680Init()
    {
        // Give time to start
        delay(10);

        if (!myBME680.begin())
        {
            Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
            while (1)
                ;
        }

        // Set up oversampling and filter initialization
        myBME680.setTemperatureOversampling(BME680_OS_8X);
        myBME680.setHumidityOversampling(BME680_OS_2X);
        myBME680.setPressureOversampling(BME680_OS_4X);
        myBME680.setIIRFilterSize(BME680_FILTER_SIZE_3);
        myBME680.setGasHeater(320, 150); // 320*C for 150 ms
    }

    //   void BME280Init()
    //   {
    //     //***Driver settings********************************//
    //     //commInterface can be I2C_MODE or SPI_MODE
    //     //specify chipSelectPin using arduino pin names
    //     //specify I2C address.  Can be 0x77(default) or 0x76

    //     //For I2C, enable the following and disable the SPI section
    //     myBME280.settings.commInterface = I2C_MODE;
    //     myBME280.settings.I2CAddress = 0x77;

    //     //For SPI enable the following and dissable the I2C section
    //     //myBME280.settings.commInterface = SPI_MODE;
    //     //myBME280.settings.chipSelectPin = 10;

    //     //***Operation settings*****************************//

    //     //renMode can be:
    //     //  0, Sleep mode
    //     //  1 or 2, Forced mode
    //     //  3, Normal mode
    //     myBME280.settings.runMode = 3; //Normal mode

    //     //tStandby can be:
    //     //  0, 0.5ms
    //     //  1, 62.5ms
    //     //  2, 125ms
    //     //  3, 250ms
    //     //  4, 500ms
    //     //  5, 1000ms
    //     //  6, 10ms
    //     //  7, 20ms
    //     myBME280.settings.tStandby = 0;

    //     //filter can be off or number of FIR coefficients to use:
    //     //  0, filter off
    //     //  1, coefficients = 2
    //     //  2, coefficients = 4
    //     //  3, coefficients = 8
    //     //  4, coefficients = 16
    //     myBME280.settings.filter = 0;

    //     //tempOverSample can be:
    //     //  0, skipped
    //     //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    //     myBME280.settings.tempOverSample = 5;

    //     //pressOverSample can be:
    //     //  0, skipped
    //     //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    //     myBME280.settings.pressOverSample = 5;

    //     //humidOverSample can be:
    //     //  0, skipped
    //     //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    //     myBME280.settings.humidOverSample = 5;

    //     Serial.begin(57600);
    //     Serial.print("Program Started\n");
    //     Serial.print("Starting BME280... result of .begin(): 0x");

    //     //Calling .begin() causes the settings to be loaded
    //     delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
    //     Serial.println(myBME280.begin(), HEX);

    //     Serial.print("Displaying ID, reset and ctrl regs\n");

    //     Serial.print("ID(0xD0): 0x");
    //     Serial.println(myBME280.readRegister(BME280_CHIP_ID_REG), HEX);
    //     Serial.print("Reset register(0xE0): 0x");
    //     Serial.println(myBME280.readRegister(BME280_RST_REG), HEX);
    //     Serial.print("ctrl_meas(0xF4): 0x");
    //     Serial.println(myBME280.readRegister(BME280_CTRL_MEAS_REG), HEX);
    //     Serial.print("ctrl_hum(0xF2): 0x");
    //     Serial.println(myBME280.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

    //     Serial.print("\n\n");

    //     Serial.print("Displaying all regs\n");
    //     uint8_t memCounter = 0x80;
    //     uint8_t tempReadData;
    //     for(int rowi = 8; rowi < 16; rowi++ )
    //     {
    //       Serial.print("0x");
    //       Serial.print(rowi, HEX);
    //       Serial.print("0:");
    //       for(int coli = 0; coli < 16; coli++ )
    //       {
    //         tempReadData = myBME280.readRegister(memCounter);
    //         Serial.print((tempReadData >> 4) & 0x0F, HEX);//Print first hex nibble
    //         Serial.print(tempReadData & 0x0F, HEX);//Print second hex nibble
    //         Serial.print(" ");
    //         memCounter++;
    //       }
    //       Serial.print("\n");
    //     }

    //     Serial.print("\n\n");

    //     Serial.print("Displaying concatenated calibration words\n");
    //     Serial.print("dig_T1, uint16: ");
    //     Serial.println(myBME280.calibration.dig_T1);
    //     Serial.print("dig_T2, int16: ");
    //     Serial.println(myBME280.calibration.dig_T2);
    //     Serial.print("dig_T3, int16: ");
    //     Serial.println(myBME280.calibration.dig_T3);

    //     Serial.print("dig_P1, uint16: ");
    //     Serial.println(myBME280.calibration.dig_P1);
    //     Serial.print("dig_P2, int16: ");
    //     Serial.println(myBME280.calibration.dig_P2);
    //     Serial.print("dig_P3, int16: ");
    //     Serial.println(myBME280.calibration.dig_P3);
    //     Serial.print("dig_P4, int16: ");
    //     Serial.println(myBME280.calibration.dig_P4);
    //     Serial.print("dig_P5, int16: ");
    //     Serial.println(myBME280.calibration.dig_P5);
    //     Serial.print("dig_P6, int16: ");
    //     Serial.println(myBME280.calibration.dig_P6);
    //     Serial.print("dig_P7, int16: ");
    //     Serial.println(myBME280.calibration.dig_P7);
    //     Serial.print("dig_P8, int16: ");
    //     Serial.println(myBME280.calibration.dig_P8);
    //     Serial.print("dig_P9, int16: ");
    //     Serial.println(myBME280.calibration.dig_P9);

    //     Serial.print("dig_H1, uint8: ");
    //     Serial.println(myBME280.calibration.dig_H1);
    //     Serial.print("dig_H2, int16: ");
    //     Serial.println(myBME280.calibration.dig_H2);
    //     Serial.print("dig_H3, uint8: ");
    //     Serial.println(myBME280.calibration.dig_H3);
    //     Serial.print("dig_H4, int16: ");
    //     Serial.println(myBME280.calibration.dig_H4);
    //     Serial.print("dig_H5, int16: ");
    //     Serial.println(myBME280.calibration.dig_H5);
    //     Serial.print("dig_H6, uint8: ");
    //     Serial.println(myBME280.calibration.dig_H6);

    //     Serial.println();
    //   }

    void printDriverError(CCS811Core::status errorCode)
    {
        switch (errorCode)
        {
        case CCS811Core::SENSOR_SUCCESS:
            Serial.print("SUCCESS");
            break;
        case CCS811Core::SENSOR_ID_ERROR:
            Serial.print("ID_ERROR");
            break;
        case CCS811Core::SENSOR_I2C_ERROR:
            Serial.print("I2C_ERROR");
            break;
        case CCS811Core::SENSOR_INTERNAL_ERROR:
            Serial.print("INTERNAL_ERROR");
            break;
        case CCS811Core::SENSOR_GENERIC_ERROR:
            Serial.print("GENERIC_ERROR");
            break;
        default:
            Serial.print("Unspecified error.");
        }
    }

    void updateOled()
    {
        // oled.clear(PAGE);

        oled.setCursor(0, 0);
        oled.setFontType(0);
        oled.print("temp:");
        // oled.print(myBME280.readTempF());
        oled.print(myBME680.temperature);

        oled.setCursor(0, 8);
        oled.setFontType(0);
        oled.print(" %rh:");
        oled.print(myBME680.humidity);

        oled.setCursor(0, 16);
        oled.setFontType(0);
        oled.print("pres: ");
        int pressureInt = (int)myBME680.pressure / 100.0;
        oled.print(pressureInt);
        oled.print("k");

        oled.setCursor(0, 24);
        oled.setFontType(0);
        oled.print("Gas: ");
        oled.print(myBME680.gas_resistance / 1000.0);
        oled.print("KOhms");

        oled.setCursor(0, 32);
        oled.setFontType(0);
        oled.print("alt: ");
        oled.print(myBME680.readAltitude(SEALEVELPRESSURE_HPA));
        oled.print("m");

        oled.setCursor(0, 40);
        oled.setFontType(0);
        oled.print("soil: ");
        oled.print(String(soilSensor1.getPercentMoist()));

        oled.display();
    }

    void printSerial()
    {
        Serial.print("Temperature: ");
        // Serial.print(myBME280.readTempC(), 2);
        Serial.println(" degrees C");

        Serial.print("Temperature: ");
        // Serial.print(myBME280.readTempF(), 2);
        Serial.println(" degrees F");

        Serial.print("Pressure: ");
        // Serial.print(myBME280.readFloatPressure(), 2);
        Serial.println(" Pa");

        Serial.print("Altitude: ");
        // Serial.print(myBME280.readFloatAltitudeMeters(), 2);
        Serial.println("m");

        Serial.print("Altitude: ");
        // Serial.print(myBME280.readFloatAltitudeFeet(), 2);
        Serial.println("ft");

        Serial.print("%RH: ");
        // Serial.print(myBME280.readFloatHumidity(), 2);
        Serial.println(" %");

        Serial.print("eCO2: ");
        // Serial.print(myCCS811.getCO2());
        Serial.println(" ppm");

        Serial.print("TVOC: ");
        // Serial.print(myCCS811.getTVOC());
        Serial.println(" ppb");

        Serial.print("SoilMoisture: ");
        Serial.print(soilSensor1.getPercentMoist());
        Serial.println(" ?");

        Serial.print("LastSoilSensorReading: ");
        Serial.print(soilSensor1.getLastSoilSensorReading());
        Serial.println("");

        if (client.loop())
        {
            Serial.println("MQTT client loop true");
        }
        if (client.isConnected())
        {
            Serial.println("MQTT client still connected");
        }

        Serial.println();
        Serial.println();
    }

    void publishData()
    {
        unsigned long currentPublishMillis = millis();

        if (currentPublishMillis - previousPublishMillis >= waitTimePublish)
        {
            //   ubidots.add("Temp", myBME280.readTempF());  // Change for your variable name
            //   ubidots.add("Relative_Humidity", myBME280.readFloatHumidity());
            //   ubidots.add("Pressure", myBME280.readFloatPressure());
            //   ubidots.add("eCO2", myCCS811.getCO2());
            //   ubidots.add("TVOC", myCCS811.getTVOC());

            //   if(ubidots.sendAll()){
            //     // Do something if values were sent properly
            //     Serial.println("Values published by device");
            //   }

            // // Particle Variables (not used)
            //  tempF = myBME280.readTempF();
            //  rh = myBME280.readFloatHumidity();

            //  // Push values to particle
            Particle.publish("temp", String(myBME680.temperature));
            Particle.publish("rh", String(myBME680.humidity));
            Particle.publish("pressure", String(myBME680.pressure / 100.0));
            // Particle.publish("soilMoisture", soilMoisture);

            String temp = "{\"temp\":";
            temp += String(myBME680.temperature);
            temp += "}";

            String rh = "{\"rh\":";
            rh += String(myBME680.humidity);
            rh += "}";

            String pressure = "{\"pressure\":";
            pressure += String(myBME680.pressure / 100.0);
            pressure += "}";

            String co2 = "{\"gas_resistance\":";
            co2 += String(myBME680.gas_resistance / 1000.0);
            co2 += "}";

            String tvoc = "{\"altitude\":";
            tvoc += String(myBME680.readAltitude(SEALEVELPRESSURE_HPA));
            tvoc += "}";

            if (!client.loop())
            {
                Serial.print("Client disconnected...");
                if (client.connect("sparkclient"))
                {
                    Serial.println("reconnected.");
                }
                else
                {
                    Serial.println("failed.");
                }
            }

            if (client.isConnected())
            {
                Serial.println("Publishing sensor data to MQTT");
                client.publish("sensor/tent11", temp);
                client.publish("sensor/tent11", rh);
                client.publish("sensor/tent11", pressure);
                client.publish("sensor/tent11", co2);
                client.publish("sensor/tent11", tvoc);
            }

            previousPublishMillis = currentPublishMillis;
        }
    }

    void updateCSS811()
    {
        //Check to see if data is available
        if (myCCS811.dataAvailable())
        {
            //Calling this function updates the global tVOC and eCO2 variables
            //   myCCS811.readAlgorithmResults();

            //   //printData fetches the values of tVOC and eCO2
            //   printSerial();

            //   float BMEtempC = myBME280.readTempC();
            //   float BMEhumid = myBME280.readFloatHumidity();

            //   Serial.print("Applying new values (deg C, %): ");
            //   Serial.print(BMEtempC);
            //   Serial.print(",");
            //   Serial.println(BMEhumid);
            //   Serial.println();

            //   //This sends the temperature data to the CCS811
            //   myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
        }
        else if (myCCS811.checkForStatusError())
        {
            Serial.println(myCCS811.getErrorRegister()); //Prints whatever CSS811 error flags are detected
        }
    }
};
GrowTent growTent1(1);

// recieve message
void callback(char *topic, byte *payload, unsigned int length)
{
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

    if (!strcmp(p, "RED"))
        RGB.color(255, 0, 0);
    else if (!strcmp(p, "GREEN"))
        RGB.color(0, 255, 0);
    else if (!strcmp(p, "BLUE"))
        RGB.color(0, 0, 255);
    else
        RGB.color(255, 255, 255);
    delay(1000);
}

/****************************************
 * Main Functions
 ****************************************/
void setup()
{
    soilSensor1.Init();
    growTent1.Init();
    growTentButton1.Init();
}

void loop()
{
    // Always run these functions - they each have their own internal timers
    soilSensor1.Update();
    growTent1.Update();
    growTentButton1.Update();
}