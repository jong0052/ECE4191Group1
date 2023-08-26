void parseStringToFloats(String input, float &wl_current, float &wr_current, float &wl_goal, float &wr_goal) {
  int index = input.indexOf("[") + 1; // Find the starting index of the values
  int endIndex = input.indexOf("]"); // Find the ending index of the values

  String valuesStr = input.substring(index, endIndex); // Extract the values between '[' and ']'
  valuesStr.replace(" ", ""); // Remove any spaces
  
  int commaIndex = valuesStr.indexOf(","); // Find the index of the first comma

  wl_current = valuesStr.substring(0, commaIndex).toFloat(); // Convert the substring to a float and assign to variable
  valuesStr = valuesStr.substring(commaIndex + 1); // Remove the parsed value and comma
  
  commaIndex = valuesStr.indexOf(",");
  wr_current = valuesStr.substring(0, commaIndex).toFloat();
  valuesStr = valuesStr.substring(commaIndex + 1);
  
  commaIndex = valuesStr.indexOf(",");
  wl_goal = valuesStr.substring(0, commaIndex).toFloat();
  valuesStr = valuesStr.substring(commaIndex + 1);
  
  wr_goal = valuesStr.toFloat(); // The remaining value is the last one
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    if (data.startsWith("Wheels")){
    float wl_current, wr_current, wl_goal, wr_goal;
    parseStringToFloats(data, wl_current, wr_current, wl_goal, wr_goal);

    wl_current = wl_current + 1;
    wr_current = wr_current + 1;
    
    Serial.print("Wheels: [");
    Serial.print(wl_current);
   Serial.print(", ");
   Serial.print(wr_current);
    Serial.print(", ");
    Serial.print(wl_goal);
   Serial.print(", ");
    Serial.print(wr_goal);
    Serial.println("]");
    }
  }
}