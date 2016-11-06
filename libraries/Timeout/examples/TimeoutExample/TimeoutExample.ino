#include "Timeout.h"

#define LED_PIN                     13
#define LED_BLINK_HALF_PERIOD_MS    500

// Global Variables
bool g_fLedOn = true;
CTimeout g_LedTimeout(LED_BLINK_HALF_PERIOD_MS);

void setup()
{
  // Initialize LED pin as an output
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    if (g_LedTimeout.Expired(true))
    {
        if (g_fLedOn)
        {
            // Turn LED on
            digitalWrite(LED_PIN, HIGH); 
        }
        else
        {
            // Turn LED off
            digitalWrite(LED_PIN, LOW); 
        }

        // Flip LED state
        g_fLedOn = !g_fLedOn;
    }
}
