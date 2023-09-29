void Serial_Log() {
//  Serial.print("  k - "); Serial.print(k);
//  Serial.print("  U "); Serial.print(Uamp);
//  Serial.print("V  Iamp "); Serial.print(Iamp);
//  Serial.print("mA  DUTY "); Serial.print(Duty * 100);
//  Serial.print("%  Error "); Serial.print(REG_Array[8]); Serial.print(" ");
//  Serial.print("  PowerSet "); Serial.print(PowerSet);
//  Serial.print("  PowerNow "); Serial.println(PowerNow);

  printValue("k", (String) k, "");
  printValue("U", (String) Uamp, "V");
  printValue("Iamp", (String) Iamp, "mA");
  printValue("DUTY", (String) (Duty * 100), "%");
  printValue("Error", (String) REG_Array[8], "");
  printValue("PowerNow", (String) PowerNow, "W");
  Serial.println();
}

void printValue(String name, String value, String unit) {
  Serial.print(name);
  Serial.print(" - ");
  Serial.print(value);
  Serial.print(unit);
  Serial.print('\t');
}
