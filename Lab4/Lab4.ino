// TODO: header

#include <Arduino_FreeRTOS.h>
#include <arduinoFFT.h>

// Acknowledgement: http://playground.arduino.cc/Main/DHTLib
#include <dht.h>

#include <queue.h>
#include <time.h>
#include "display.h"

// defines
#define DEMO 0 // set to 1 for demo (enables song/FFT tasks)

#define LED_EXTERNAL 53
#define SPEAKER_PIN 6
#define TEMP_SENSOR_PIN 13
#define FAN_PIN 52

#define E 659 // Hz
#define C 523 // Hz
#define G 784 // Hz
#define g 392 // Hz
#define R 0 // Rest, play no sound

#define SONG_MELODY_LENGTH 21
#define CLOCK_SPEED_SCALED 62500

#define FFT_SAMPLE_SIZE 128
#define FFT_SAMPLING_FREQUENCY 5000

// variables
const int song[] = {E, R, E, R, R, E, R, R, C, R, E, R, R, G, R, R, R, R, R, g, R};
QueueHandle_t randomQueue;
QueueHandle_t statusQueue;
dht DHT;
static int temp;
volatile int sensorValue;
volatile int step;

QueueHandle_t sensorQueue;

// function declarations
void TaskRt1(void *pvParameters);
void setFrequency(int frequency);

void setup() {
  Serial.begin(19200);

  // seed the random
  srand(time(NULL));

  // timer stuff for speaker
  TCCR4A = 0;     // set control registers to 0 initiallty
  TCCR4B = 0;     // set control registers to 0 initiallty

  TCCR4A |= ((1 << WGM40) | (1 << WGM41) | (1 << COM4A0)); // set control bits to 1 for Fast PWM mode
  TCCR4B |= ((1 << WGM42) | (1 << WGM43));                 // set control bits to 1 for Fast PWM mode

  TCNT4 = 0; // reset timer value to 0

  noInterrupts(); // disable interrupts

  setFrequency(0);

  TCCR4B |= ((1 << CS42)); // turn timer on
  // end timer stuff for speaker



  // timer stuff for scheduler
  noInterrupts();

  TCCR2A = 0;
  TCCR2B = 0;

  OCR2B = 500; // 0.002/(1/(16000000/128))
  TCCR2B |= ((1 << CS22) | (1 << CS20)); // set control bits for 128 prescaler

  TIMSK2 |= (1 << OCIE1B);               // enable the interrupt

  interrupts();                          // enable interrupts




  // initialize display pins
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);

  // initialize other pins
  pinMode(FAN_PIN, OUTPUT);

  
  // task stuff

  xTaskCreate(
    TaskRt1
    ,  "RT-1"
    ,  256  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );

  
  if (DEMO == 1) {
    xTaskCreate(
      TaskRt2
      ,  "RT-2"
      ,  128  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL );

    xTaskCreate(
      TaskRt3p0
      ,  "RT-3p0"
      ,  2048  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL );
    
    xTaskCreate(
      TaskRt4
      ,  "RT-4"
      ,  2048  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL );
  }
  
  xTaskCreate(
      TaskGetTemp
      ,  "TaskGetTemp"
      ,  256  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL );
  
  xTaskCreate(
      TaskReadPotentiometer
      ,  "TaskReadPotentiometer"
      ,  256  // Stack size
      ,  NULL
      ,  1  // Priority
      ,  NULL );

  sensorQueue = xQueueCreate(
    1,
    sizeof(int)
  );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  //  (note how the above comment is WRONG!!!)
  vTaskStartScheduler();
}

void loop() {
}

// TODO: comment
ISR(TIMER2_COMPB_vect) {
  step++;
  
  if (step == 1) {
    chooseDigitOnDisplay(0);
    showSingleDigit((int)((sensorValue / 1) % 10));
  } else if (step == 2) {
    chooseDigitOnDisplay(1);
    showSingleDigit((int)((sensorValue / 10) % 10));
  } else if (step == 3) {
    chooseDigitOnDisplay(2);
    showSingleDigit((int)((sensorValue / 100) % 10));
  } else {
    chooseDigitOnDisplay(3);
    showSingleDigit((int)((sensorValue / 1000) % 10));

    step = 0;
  }
}

// TODO: comment
void TaskRt1(void *pvParameters) {
  // initialize external LED as output
  pinMode(LED_EXTERNAL, OUTPUT);

  for (;;) {
    digitalWrite(LED_EXTERNAL, HIGH);   // turn the LED on
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100ms
    digitalWrite(LED_EXTERNAL, LOW);    // turn the LED off
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for 200ms
  }
}

// TODO: comment
void TaskRt2(void *pvParameters) {
  static int note;
  static int runs;

  // make speaker output
  pinMode(SPEAKER_PIN, OUTPUT);

  for (;;) {
    // every 100ms, and within bounds of melody
    if (note < SONG_MELODY_LENGTH) {
      setFrequency(song[note]);
      note++;

      vTaskDelay( 100 / portTICK_PERIOD_MS );
    }
    // passed end so loop back around
    // (note: last note is a rest, so speaker is already emitting no sound)
    else {
      note = 0;
      runs++;

      // if it's run 3 times, quit it
      if (runs == 3) {
        vTaskDelete(NULL);
      } else {
        // otherwise, wait 1.5s
        vTaskDelay( 1500 / portTICK_PERIOD_MS );
      }
    }
  }
}

// TODO: comment
void TaskRt3p0(void *pvParameters) {
  double randomSamples[FFT_SAMPLE_SIZE];

  // never return
  for(;;) {
    // fill the array with random data
    for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
      randomSamples[i] = ((double)rand() / (double)RAND_MAX);
    }

    // initialize the queue
    randomQueue = xQueueCreate(
      FFT_SAMPLE_SIZE,
      sizeof(double)
    );

    // start RT3p1
    xTaskCreate(
      TaskRt3p1
      ,  "RT-3p1"
      ,  128  // Stack size
      ,  &randomSamples
      ,  2  // Priority
      ,  NULL );

    // halt itself
    vTaskDelete(NULL);
  }
}

// TODO: comment
void TaskRt3p1(void *pvParameters) {
  for(;;) {
    // initialize the queue for signaling
    statusQueue = xQueueCreate(
      5,
      sizeof(int)
    );

    for (int i = 0; i < 5; i++) {
      // send the data to task 4
      xQueueSendToBack(
        randomQueue,
        (double*)pvParameters,
        1
      );

      // wait for it to signal it is done
      int status;
      while(1) {
        if (xQueueReceive(statusQueue, &status, 1) == pdPASS) {
          Serial.print("done in: ");
          Serial.print(status);
          Serial.println("ms");
          break;
        }
      }
    }

    // halt itself
    vTaskDelete(NULL);
  }
}

// TODO: comment
void TaskRt4(void *pvParameters) {
  double samples[FFT_SAMPLE_SIZE];

  for(;;) {
    int start = millis();

    while(1) {
      if (xQueueReceive(randomQueue, &samples, 1) == pdPASS) {
        arduinoFFT FFT;

        double vImag[FFT_SAMPLE_SIZE];
        for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
          vImag[i] = 0.0;
        }

        // construct and compute FFT
        FFT = arduinoFFT(samples, vImag, FFT_SAMPLE_SIZE, FFT_SAMPLING_FREQUENCY);
        FFT.Compute(FFT_FORWARD);
        
        // signal completion
        int time = millis() - start;
        xQueueSendToBack(
          statusQueue,
          &time,
          1
        );
        break;
      }
    }
  }
}

// TODO: comment
void TaskGetTemp(void *pvParameters) {
  // how many cycles temperature has been below sensor value
  // (needed because temperature sensor sometimes reads
  // incorrect values [too big or small] randomly)
  static int cyclesBelowCutoff = 0;

  for(;;) {
    int sensorVal;
    if (xQueueReceive(sensorQueue, &sensorVal, 1) == pdPASS) {
      DHT.read11(TEMP_SENSOR_PIN);
      int tempTemp = (int)DHT.temperature;

      // only show the temp if it is >=0
      // (<0 indicates error here)
      if (tempTemp >= 0) {
        temp = tempTemp;

        // check if the temp is above cutoff
        if (tempTemp > sensorVal) {
          digitalWrite(FAN_PIN, HIGH);

          cyclesBelowCutoff = 0;
        } else {
          cyclesBelowCutoff++;

          // only turn it off if we've been below temperature
          // cutoff for 3 consecutive cycles
          if (cyclesBelowCutoff > 3) {
            digitalWrite(FAN_PIN, LOW);
          }
        }
      }
    }
  }
}

// TODO: comment
void TaskReadPotentiometer(void *pvParameters) {
  for(;;) {
    // read it in (0-1023), dividing by 10
    // to reduce it to max ~102
    int sensorVal = analogRead(A0) / 10;
    sensorValue = sensorVal;

    // send sensor reading to other task
    xQueueSendToBack(
      sensorQueue,
      &sensorVal,
      1
    );

    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

/*****************************************
 * [setFrequency]
 * 
 * frequency (int) - frequency to set speaker to
 * Returns - nothing (void)
 * 
 * Sets the corresponding registers of Timer4 to
 * the necessary values for the given frequency
 * 
 * Author [Jake Short, Vincent Kwok]
 * 
 * Acknowledgements:
 * https://courses.cs.washington.edu/courses/cse474/23sp/assignments/ATmega2560_datasheet.pdf
 */
void setFrequency(int frequency) {
  if (frequency == 0) {
    TCCR4B &= ~(1 << CS42); // turn timer off
  }
  else {
    int regVal = (CLOCK_SPEED_SCALED / 2) / frequency;
    OCR4A = regVal;
    OCR4B = regVal / 2; // 50%
    TCCR4B |= (1 << CS42); // turn timer on
  }
}