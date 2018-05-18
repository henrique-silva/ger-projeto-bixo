#include <Servo.h>
#include <QTRSensors.h>
#include <Ultrasonic.h>
#include <DCMotor.h>
#include <TimerOne.h>

/* Ultrassonic sensor */
#define TRIGGER_PIN             3
#define ECHO_PIN                2

/* Reflectance sensor */
#define NUM_SENSORS             6       // number of sensors used
#define TIMEOUT                 2500    // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN             A5       // emitter is controlled by digital pin 2
#define QTR_CENTER_POS          3000
#define SENSOR_1_PIN            A0
#define SENSOR_2_PIN            A1
#define SENSOR_3_PIN            A2
#define SENSOR_4_PIN            A3
#define SENSOR_5_PIN            A4

/* Claw definitions */
#define CLAW_PIN                12
#define CLAW_OPEN_ANGLE         0
#define CLAW_CLOSED_ANGLE       50

/* DC Motors */
#define MOTOR_RIGHT_SPEED_PIN   11
#define MOTOR_RIGHT_ENA         7
#define MOTOR_RIGHT_ENB         8

#define MOTOR_LEFT_SPEED_PIN    6
#define MOTOR_LEFT_ENA          5
#define MOTOR_LEFT_ENB          10

#define MOTOR_MAX_SPEED         255
#define MOTOR_SLOW_SPEED        100

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
volatile float object_dist = 0;

// QTR reflectance sensors
QTRSensorsRC qtrrc((unsigned char[]) {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN, SENSOR_5_PIN}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

/* Variables to hold the reflectance sensors read */
unsigned int sensorValues[NUM_SENSORS];
volatile unsigned int line_pos = 0;

/* Claw attached to servo */
Servo claw;

/* Motor objects */
DCMotor motor_left(MOTOR_LEFT_ENA,MOTOR_LEFT_ENB, MOTOR_LEFT_SPEED_PIN);
DCMotor motor_right(MOTOR_RIGHT_ENA,MOTOR_RIGHT_ENB, MOTOR_RIGHT_SPEED_PIN);

void qtr_calibrate( void )
{
    Serial.println("Calibrating reflectance sensors...");

    for (int i = 0; i < 200; i++) {  // make the calibration take about 5 seconds
        delay(15);
        qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }

    // print the calibration minimum values measured when emitters were on
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(qtrrc.calibratedMinimumOn[i]);
        Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(qtrrc.calibratedMaximumOn[i]);
        Serial.print(' ');
    }
    Serial.println();
}

void setup()
{
    Serial.begin(9600);

    Serial.println("Starting sensors and motors!");
    /* Initiate claw */
    claw.attach(CLAW_PIN);
    claw.write(CLAW_OPEN_ANGLE);

    /* Stop the motors */
    motor_right.stop();
    motor_left.stop();

    /* Calibrate reflectance sensors */
    qtr_calibrate();
}

int left_speed = MOTOR_MAX_SPEED;
int right_speed = MOTOR_MAX_SPEED;

void loop()
{
    /* Read distance from ultrassonic sensor */
    object_dist = ultrasonic.convert(ultrasonic.timing(), Ultrasonic::CM);

    Serial.print("Object distance: ");
    Serial.println(object_dist);

    /* Read reflectance sensors and get line position estimative (0 - 4000) */
    line_pos = qtrrc.readLine(sensorValues);
    Serial.print("Line position:");
    Serial.println(line_pos);

    /* Update motor speed */
    motor_left.setOutput(left_speed);
    motor_right.setOutput(right_speed);

#if 0
    if (object_dist <= 5) {

        /* Stop the motor */
        Serial.println("stopping motors");
        motor_left.stop();
        motor_right.stop();

        /* Grab object */
        Serial.println("grabbing object");
        delay(1000);
        claw.write(CLAW_CLOSED_ANGLE);

        /* Turn around */
        Serial.println("turning around");
        motor_left.goForward();
        motor_right.goBackward();
        delay(1000);

        motor_left.stop();
        motor_right.stop();

        /* Drop object */
        claw.write(CLAW_OPEN_ANGLE);

        delay(1000);
        motor_right.goBackward();
        motor_left.goBackward();
        delay(1000);

        motor_left.stop();
        motor_right.stop();

        /* Turn back */
        Serial.println("turning back");
        motor_left.goBackward();
        motor_right.goForward();
        delay(1000);

        /* Stop the motor */
        motor_left.stop();
        motor_right.stop();
    }
#endif
#if 1
    if (line_pos < 1500) {
        right_speed = MOTOR_SLOW_SPEED;
    }
    if (line_pos > 2500) {
        left_speed = MOTOR_SLOW_SPEED;
    }
    if (line_pos > 1500 && line_pos < 2700) {
        left_speed = MOTOR_MAX_SPEED;
        right_speed = MOTOR_MAX_SPEED;
    }

#endif
}
