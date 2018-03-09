/*
Ultrasonic Sensor HC-SR04
Vietnamese-German University
By Tung Le Vo
Calculate distance to obstacles with SRF04
Last Modified 8th March 2018
*/

float getDistance(int ultrasonicTriggerPin, int ultrasonicEchoPin) {
    /*
        Send ultrasonic signal to bounce off objects
     */
    digitalWrite(ultrasonicTriggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicTriggerPin, LOW);

    unsigned long duration = pulseIn(ultrasonicEchoPin, HIGH); // Get returning pulses
    unsigned long distance = duration/29/2; // Convert miliseconds to centimeters

    return distance;
}
