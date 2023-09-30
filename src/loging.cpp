#include "loging.h"

void printValue(String name, String value, String unit) {
    Serial.print(name);
     Serial.print(" - ");
    Serial.print(value);
    Serial.print(unit);
    Serial.print('\t');
}

void Serial_Log(byte k, unsigned int Uamp, unsigned int Iamp, double Duty, byte error, unsigned int PowerNow) {
    printValue("k", (String) k, "");
    printValue("U", (String) Uamp, "V");
    printValue("Iamp", (String) Iamp, "mA");
    printValue("DUTY", (String) (Duty * 100), "%");
    printValue("Error", (String) error, "");
    printValue("PowerNow", (String) PowerNow, "W");
    Serial.println();
}

