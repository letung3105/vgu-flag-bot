/**
 * Ultrasonic Sensor HC-SR04
 * Vietnamese-German University
* By Tung Le Vo
 * Calculate distance to obstacles with HC-SR04
 * Last Modified 11th April 2018
 */


// Calculate the distance
// Since 1 pin is used for both the echo and trigger
// we have to change the pin mode of it accordingly
uint16_t getDistance(uint8_t mySensor) {
    // Sending out sound waves
    pinMode(mySensor, OUTPUT);
    digitalWrite(mySensor, LOW);
    delayMicroseconds(2);
    digitalWrite(mySensor, HIGH);
    delayMicroseconds(10);
    digitalWrite(mySensor, LOW);

    pinMode(mySensor, INPUT);
    uint16_t duration = pulseIn(mySensor, HIGH); // Return pulses duration (ms)
    uint16_t distance = duration * 0.034 / 2; // Convert microseconds to centimeters

    return distance;
}


// Simple function to compare distance calculated from 2 different sensors
bool detectObstacle(uint16_t threshold){
    if ((getDistance(leftUltraSonicSensor) <= threshold) && (getDistance(rightUltraSonicSensor) <= threshold)){
        return true;
    }
    return false;
}
