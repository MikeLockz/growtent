// This #include statement was automatically added by the Particle IDE.
#include "SparkFun_VEML6075.h"

// This #include statement was automatically added by the Particle IDE.
//#include <SparkFun-VEML6075.h> // might cause duplicate declration error if this ever gets linked

// This #include statement was automatically added by the Particle IDE.
#include <clickButton.h>

// This #include statement was automatically added by the Particle IDE.
#include <MQTT.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_BME680.h>

// This #include statement was automatically added by the Particle IDE.
#include <I2cMaster.h>

// This #include statement was automatically added by the Particle IDE.
#include "stdint.h"

// This #include statement was automatically added by the Particle IDE.
#include <SparkFunMicroOLED.h>
#include "math.h"

/****************************************
 * Define Constants
 ****************************************/

#define SEALEVELPRESSURE_HPA (1013.25) // Sea level pressure for BME680

//long fiveMinutes = 5 * 60 * 1000UL;
long fiveMinutes = 1 * 30 * 1000UL;

/****************************************
 * Initialize Global Objects
 ****************************************/

MicroOLED oled;
Adafruit_BME680 myBME680;
VEML6075 myVEML6075;

byte server[] = {192, 168, 0, 24};
MQTT client(server, 1883, callback);

/****************************************
 * Auxiliary Functions
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

        //Initialize BME680
        BME680Init();

        //Initialize
        VEML6075Init();
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

    void VEML6075Init()
    {
        // the VEML6075's begin function can take no parameters
        // It will return true on success or false on failure to communicate
        if (myVEML6075.begin() == false)
        {
            Serial.println("Unable to communicate with VEML6075.");
            while (1)
                ;
        }
        Serial.println("UVA, UVB, UV Index");
    }

    void updateOled()
    {
        // oled.clear(PAGE);

        oled.setCursor(0, 0);
        oled.setFontType(0);
        oled.print("temp:");
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
        // oled.print("soil: ");
        // oled.print(String(soilSensor1.getPercentMoist()));
        oled.print("uvab: ");
        oled.print(String(myVEML6075.uva()) + ", " + String(myVEML6075.uvb()) + ", " + String(myVEML6075.index()));

        oled.display();
    }

    void printSerial()
    {
        Serial.print("Temperature: ");
        Serial.print(myBME680.temperature, 2);
        Serial.println(" degrees C");

        Serial.print("Pressure: ");
        Serial.print(myBME680.pressure / 100.0, 2);
        Serial.println(" Pa");

        Serial.print("Altitude: ");
        Serial.print(myBME680.readAltitude(SEALEVELPRESSURE_HPA), 2);
        Serial.println("m");

        Serial.print("%RH: ");
        Serial.print(myBME680.humidity, 2);
        Serial.println(" %");

        Serial.print("Gas resistance: ");
        Serial.print(myBME680.gas_resistance / 1000.0, 2);
        Serial.println(" ppm");

        Serial.print("UVA: ");
        Serial.println(myVEML6075.uva());

        Serial.print("UVB: ");
        Serial.println(myVEML6075.uvb());

        Serial.print("UV index: ");
        Serial.println(myVEML6075.index());

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
            //  // Push values to particle
            Particle.publish("temp", String(myBME680.temperature));
            Particle.publish("rh", String(myBME680.humidity));
            Particle.publish("pressure", String(myBME680.pressure / 100.0));
            Particle.publish("altitude", String(myBME680.readAltitude(SEALEVELPRESSURE_HPA)));
            Particle.publish("gas resistance", String(myBME680.gas_resistance / 1000.0));
            // Particle.publish("soilMoisture", soilMoisture);
            Particle.publish("uva", String(myVEML6075.uva()));
            Particle.publish("uvb", String(myVEML6075.uvb()));
            Particle.publish("uv index", String(myVEML6075.index()));

            String temp = "{\"temp\":";
            temp += String(myBME680.temperature);
            temp += "}";

            String rh = "{\"rh\":";
            rh += String(myBME680.humidity);
            rh += "}";

            String pressure = "{\"pressure\":";
            pressure += String(myBME680.pressure / 100.0);
            pressure += "}";

            String gas_resistance = "{\"gas_resistance\":";
            gas_resistance += String(myBME680.gas_resistance / 1000.0);
            gas_resistance += "}";

            String altitude = "{\"altitude\":";
            altitude += String(myBME680.readAltitude(SEALEVELPRESSURE_HPA));
            altitude += "}";

            String uva = "{\"uva\":";
            uva += String(myVEML6075.uva());
            uva += "}";

            String uvb = "{\"uvb\":";
            uva += String(myVEML6075.uvb());
            uva += "}";

            String uv_index = "{\"uv_index\":";
            uva += String(myVEML6075.index());
            uva += "}";

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
                client.publish("sensor/tent11", gas_resistance);
                client.publish("sensor/tent11", altitude);
                client.publish("sensor/tent11", uva);
                client.publish("sensor/tent11", uvb);
                client.publish("sensor/tent11", uv_index);
            }

            previousPublishMillis = currentPublishMillis;
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