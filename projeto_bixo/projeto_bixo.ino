#include <Servo.h>
#include <QTRSensors.h>
#include <Ultrasonic.h>
#include <DCMotor.h>
#include <TimerOne.h>

/* Ultrassonic sensor */
#define TRIGGER_PIN		3
#define ECHO_PIN		2

/* Reflectance sensor */
#define NUM_SENSORS		6	// number of sensors used
#define TIMEOUT			2500	// waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN		4	// emitter is controlled by digital pin 2
#define QTR_CENTER_POS          3000
#define SENSOR_1_PIN            5
#define SENSOR_2_PIN            6
#define SENSOR_3_PIN            7
#define SENSOR_4_PIN            8
#define SENSOR_5_PIN            9

/* Claw definitions */
#define CLAW_PIN		10
#define CLAW_OPEN_ANGLE		0
#define CLAW_CLOSED_ANGLE	50

/* DC Motors */
#define MOTOR_RIGHT_SPEED_PIN   1
#define MOTOR_RIGHT_ENA         1
#define MOTOR_RIGHT_ENB         1

#define MOTOR_LEFT_SPEED_PIN    1
#define MOTOR_LEFT_ENA          1
#define MOTOR_LEFT_ENB		1

#define MOTOR_MAX_SPEED         200

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
volatile float object_dist;

// QTR sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN, SENSOR_5_PIN}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
static unsigned int line_pos;

Servo claw;

DCMotor motor_left(MOTOR_RIGHT_ENA, MOTOR_RIGHT_ENB, MOTOR_RIGHT_SPEED_PIN);
DCMotor motor_right(MOTOR_LEFT_ENA, MOTOR_LEFT_ENB, MOTOR_LEFT_SPEED_PIN);

void qtr_calibrate( void )
{
    Serial.println("Calibrating reflectance sensors...");

    for (int i = 0; i < 200; i++) {  // make the calibration take about 5 seconds
        motor_left.setOutput(MOTOR_MAX_SPEED);
        motor_right.setOutput(-1 * MOTOR_MAX_SPEED);
        delay(100);

        motor_left.stop();
        motor_right.stop();

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

    /* Initiate claw */
    claw.attach(CLAW_PIN);
    claw.write(CLAW_OPEN_ANGLE);

    /* Stop the motors */
    motor_left.stop();
    motor_left.setOutput(0);
    motor_right.stop();
    motor_right.setOutput(0);

    /* Calibrate reflectance sensors */
    qtr_calibrate();
}

void loop()
{
    /* Read distance from object using the ultrasonic sensor and use average it with the last result */
    object_dist += ultrasonic.convert(ultrasonic.timing(), Ultrasonic::CM);
    object_dist = object_dist/2;
    Serial.println("Object distance: %f", object_dist);
    
    /* Read reflectance sensors and get line position estimative (0 - 4000) */
    line_pos = qtrrc.readLine(sensorValues);
    Serial.println("Line position: %d", line_pos);

    
    if (object_dist <= 5.0) {
        /* Lower motor speed */
        motor_left.setOutput(MOTOR_MAX_SPEED/2);
        motor_right.setOutput(MOTOR_MAX_SPEED/2);

        /* Wait until the claw gets closer to the object */
        while (object_dist > 3.3 );
        /* Stop the motor */
        motor_left.stop();
        motor_right.stop();

        /* Grab object */
        claw.write(CLAW_CLOSED_ANGLE);

        /* Turn around */
	motor_left.setOutput(MOTOR_MAX_SPEED);
	motor_right.setOutput(-1 * MOTOR_MAX_SPEED);
	delay(2500);

        /* Drop object */
        claw.write(CLAW_OPEN_ANGLE);

        /* Turn back */
	motor_left.setOutput(-1 * MOTOR_MAX_SPEED);
	motor_right.setOutput(MOTOR_MAX_SPEED);
	delay(2500);

        /* Stop the motor */
        motor_left.stop();
        motor_right.stop();

    }

//    if (line_pos > 2300) {
//	float rel = (((float)line_pos/(float)QTR_CENTER_POS)-1.0);
//    }
}
