#include <Arduino.h>
extern "C" {
//   #include "pico/stdlib.h"
  #include "esl_math.h"
  #include "adcs_model.h"
  #include "lunar_adcs_model.h"
}

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 50
#endif

int led_pin = LED_BUILTIN;

// ---- Arduino-style entry points ----
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }  // wait for USB serial

    pinMode(led_pin, OUTPUT);
}

const size_t DOUBLES_TO_READ = 11;
const size_t BYTES_TO_READ   = DOUBLES_TO_READ * sizeof(double);

static double u[DOUBLES_TO_READ];

double jd, dt;
Matrix ICRF2ME, ICRF2ORC;
Vector3 Sat2Tgt, latlonalt, TGT_ref, currentState_x, SUN_pos, Earth_pos, sun_orc, earth_orc;
double currentState_xv[6], previousState_xv[6], osculating_elements[12], y[40];

double start, elapsed_propagator, elapsed_models;

void loop() {
    // If serial data available, read a full line
    if (Serial.available() >= BYTES_TO_READ + 2) {
        size_t bytesRead = Serial.readBytes((char*)u, BYTES_TO_READ);

        // Read terminator
        char terminator[2];
        size_t termRead = Serial.readBytes(terminator, 2);

        bool goodFrame = (bytesRead == BYTES_TO_READ &&
                          termRead == 2 &&
                          terminator[0] == '\r' &&
                          terminator[1] == '\n');

        if (goodFrame) {
            // ✅ Good frame
            digitalWrite(led_pin, HIGH);

            // Processing
            jd = u[0];
            previousState_xv[0] = u[1]; previousState_xv[1] = u[2]; previousState_xv[2] = u[3];
            previousState_xv[3] = u[4]; previousState_xv[4] = u[5]; previousState_xv[5] = u[6];
            dt = u[7];
            TGT_ref.x = u[8] * DEG2RAD;
            TGT_ref.y = u[9] * DEG2RAD;
            TGT_ref.z = u[10] + RAD_M;

            ICRF2ME = LUNAR_ADCS_MODEL_ICRF2ME(jd);
            LUNAR_ADCS_MODEL_LunarPropagator(previousState_xv, ICRF2ME, 10, 10, MU_M, RAD_M, dt, currentState_xv);
            LUNAR_ADCS_MODEL_OsculatingElements(currentState_xv, osculating_elements, previousState_xv);
            latlonalt = LUNAR_ADCS_MODEL_LatLonAlt(currentState_xv, ICRF2ME);
            ICRF2ORC = LUNAR_ADCS_MODEL_ICRF2ORC(currentState_xv);
            currentState_x.x = currentState_xv[0]; currentState_x.y = currentState_xv[1]; currentState_x.z = currentState_xv[2];
            Sat2Tgt = LUNAR_ADCS_MODEL_Sat2Tgt(TGT_ref, currentState_x, ICRF2ME, ICRF2ORC);
            Earth_pos = LUNAR_ADCS_MODEL_EarthOrbit(jd);
            SUN_pos = LUNAR_ADCS_MODEL_SunOrbit(jd, Earth_pos);
            sun_orc = MATRIX_MultiplyVector(ICRF2ORC, VECTOR3_normalize(VECTOR3_sub(SUN_pos, currentState_x)));
            earth_orc = MATRIX_MultiplyVector(ICRF2ORC, VECTOR3_normalize(VECTOR3_sub(Earth_pos, currentState_x)));

            y[0]  = osculating_elements[6]; y[1] = osculating_elements[8]; y[2] = osculating_elements[5]; y[3] = osculating_elements[7];
            y[4]  = latlonalt.x; y[5] = latlonalt.y; y[6] = latlonalt.z;
            y[7] = currentState_xv[0]; y[8] = currentState_xv[1]; y[9] = currentState_xv[2];
            y[10] = currentState_xv[3]; y[11] = currentState_xv[4]; y[12] = currentState_xv[5];
            y[13] = ICRF2ME.data[0][0]; y[14] = ICRF2ME.data[0][1]; y[15] = ICRF2ME.data[0][2];
            y[16] = ICRF2ME.data[1][0]; y[17] = ICRF2ME.data[1][1]; y[18] = ICRF2ME.data[1][2];
            y[19] = ICRF2ME.data[2][0]; y[20] = ICRF2ME.data[2][1]; y[21] = ICRF2ME.data[2][2];
            y[22] = Sat2Tgt.x; y[23] = Sat2Tgt.y; y[24] = Sat2Tgt.z;
            y[25] = SUN_pos.x; y[26] = SUN_pos.y; y[27] = SUN_pos.z;
            y[28] = Earth_pos.x; y[29] = Earth_pos.y; y[30] = Earth_pos.z;
            y[31] = sun_orc.x; y[32] = sun_orc.y; y[33] = sun_orc.z;
            y[34] = earth_orc.x; y[35] = earth_orc.y; y[36] = earth_orc.z;
            y[37] = osculating_elements[9]; y[38] = osculating_elements[10]; y[39] = osculating_elements[11];
            y[40] = elapsed_propagator; y[41] = elapsed_models;

        } else {
            // ❌ Timeout / incomplete frame, resync
            for (int i = 0; i < sizeof(y)/sizeof(y[0]); i++) y[i] = -1.0;

            // flush until newline to resync
            while (Serial.available()) {
                if (Serial.read() == '\n') break;
            }
        }

        // Send response
        uint8_t header[2] = {0xAA, 0x55};
        Serial.write(header, 2);
        Serial.write((uint8_t*)&y, sizeof(y));
        Serial.write("\r\n");  // terminate line with CR+LF

        digitalWrite(led_pin, LOW);
    }
}
