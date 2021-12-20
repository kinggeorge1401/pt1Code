/*==================================================================================
Version 02.05 of ICU 

 Bachelor thesis by Tobias Augustin
 Pins:
 Button A - Pin 32
 Button B - Pin 34
 Button Master - Pin 35
 Neopixel LED Strip - Pin 25
 Buzzer - Pin 18
 SCL - Pin 22
 SDA - Pin 21
 MOSI - Pin 23  
 MISO - Pin 19
 CE - 12
 CSS - 5
 Used Microcontroller - ESP32 WROOM



====================================================================================*/

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(12,5); // CE, CSN         
const byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int i = 0;
char tmp;
boolean button_state = 0;

// Dual Core features

TaskHandle_t Task1;
TaskHandle_t Task2;
//

// Which pin on the Microcontroller is connected to the NeoPixels?
#define PIN        25 

// How many NeoPixels are attached to the MC?
#define NUMPIXELS 8 // Number of Pixels in the Neopixel LED Strip

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Pins
int buzzer = 18;
int step = 0;
int zc = 0; 
int lHP = 1;
// Constants for button Pins
const int button1 = 32;
const int button2 = 34;
const int button3 = 35;
const int button4 = 16;
const int button5 = 15;


//int status
int signal_stat= 0;
int light_stat= 0;
int noMove = 0;
int masterPush = 0;
int opStat = 0;
int opStat_prev = 0;
int BPush = 0;

// Boolean values that define startup process
bool initialized = false; // If true, heading is initialized
bool initSteplen = true; // If false, step length is initialized
bool calibrated = true; // If true, the device is not being calibrated during startup
bool mute = false; // If true, buzzer is muted

//Variables
int blinkInterval = 1000; //millis
double deltaMillis3 = 0;
double prevMillis3 = 0;
double last_move = 0;
double prevMillis4 = 0;
double deltaMillis4 = 0;
double timePushed=0;
double prevPushed = 0;
double sincePushed = 0;
int blink = 0;
int prev_blink = 1;
int trgr = 0;


double stepln = 0.83;
double posx = 0;
double posy = 0;
double Heading = 0;
double Heading_Prio = 0;
double initHeading = 0;

int samplingC;

// Function prototypes
void signalLightStatus(int status);
double lowpassFiltering(double factor, double accF, double acc_prioF);
void signal(int status);
void InitSteplen();
void readSensorData();
void testHMI_features();

//Position values and sample rate delays
double xPos = 0, yPos = 0, headingVel = 0, Vel_x = 0,Vel_y = 0;
double xPosG = 0, yPosG = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 10; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample


//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void)
{
  Serial.begin(115200); //Initialize Serial port
  
  pinMode(button1,INPUT); //Set button Pins to input
  pinMode(button2,INPUT);
  pinMode(button3,INPUT);
  pinMode(button4,INPUT);
  pinMode(button5,INPUT);
  
  // Check if the mute pin is set. This is performed at program start
  if(digitalRead(button4)==1){
			mute=true;
		}
		
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  
  
  
    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
					
  delay(500);
  
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }


  delay(1000);

  
  
  
    


}
// Task1code Main Loop
void Task1code( void * pvParameters ){
	
	
  uint8_t system, gyro, accel, mag = 0; //Variables for calibration

  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening(); 
 
  if(calibrated == false){
    signalLightStatus(2);
    
	  while(system != 3) //if system value is 3 (calibrated) end process
   { 
       bno.getCalibration(&system, &gyro, &accel, &mag); // Start bno055 calibration and read calibration values
   
       Serial.print("CALIBRATION: Sys="); 
       Serial.print(system, DEC); 
       Serial.print(" Gyro="); 
       Serial.print(gyro, DEC); 
       Serial.print(" Accel="); 
       Serial.print(accel, DEC); 
       Serial.print(" Mag="); 
       Serial.println(mag, DEC);  
	  

	  
       delay(100); 
   }  
   calibrated = true; 
  }
   
  Serial.println(""); Serial.println("Calibrated");
  
  //set LEDs to green to signalize successfull calibration
  signalLightStatus(1);
  signal(1);
  delay(1000);
  
  
  for(;;){ // Loop of task 1
  unsigned long tStart = micros();
  
  if(opStat!=6){
	  if(initSteplen==false){ //Checks if step length has to be initialized
	  InitSteplen();
  }
  readSensorData(); //Function that reads and interprets sensor data

  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
  
  }
  else{
	  signal(3);
  }
  
}
}

//Task2code: Handles HMI Features
void Task2code( void * pvParameters ){
    #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
    #endif

 for(;;){
	if(opStat <= 5){ // Loop handels button presses as long as the operational status is not greater than five (no emergency)
		
		deltaMillis4 = millis() - prevMillis4;
		
		if(digitalRead(button1)==1 && deltaMillis4>=200){// Is Button A pushed?
			Serial.print("A Button Push \n");
			signalLightStatus(1);   // Send the updated pixel colors to the hardware.
      
	
			prevMillis4 = millis();
			signal(1);
		}
		
		
		
		if(digitalRead(button2)==1 && deltaMillis4>=200){ // Is Button B pushed?
			Serial.print("B Button Push \n");
			
			signalLightStatus(2);
      
			BPush = BPush+1;
			prevMillis4 = millis();
			

      if(BPush >=2 && deltaMillis4<=500){ //If B Button is pushed twice in 1.5 s, the operational status is changed
       Serial.print("Double Push B \n");
	   if(opStat>=1){
		   opStat = opStat-1;
	   }
       BPush= 0;
     }
			
		signal(1); 
		}
		
		
		
		if(digitalRead(button3)==1 && deltaMillis4>=200){
			if(masterPush==0){
				timePushed=millis();
		
			}
			masterPush=1;
			if(opStat<=5){
				opStat=opStat+1;
			}
			
			Serial.print("Master Push \n");
			for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

			// pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
			// Here we're using a moderately bright green color:
			pixels.setPixelColor(i, pixels.Color(50, 0, 0));

			pixels.show();   // Send the updated pixel colors to the hardware.
			}
			prevMillis4 = millis();
			
				
			sincePushed = millis()-timePushed;
			
			if(masterPush==1 && sincePushed >=500){
				Serial.print("LongPush \n");
				opStat_prev = opStat;
				opStat=6;
				masterPush=0;
				sincePushed = 0;
				for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

			// pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
			// Here we're using a moderately bright green color:
				pixels.setPixelColor(i, pixels.Color(180, 180, 180));

				pixels.show();   // Send the updated pixel colors to the hardware.
				}
				delay(500);
			}
			
			
		}
		
		if(digitalRead(button3)==0){
			masterPush=0;
			timePushed=0;
			sincePushed=0;
		}
		
		if(digitalRead(button4)==1){
			//does something
		}
		
		if(digitalRead(button5)==1 && trgr == 0){
			xPosG = 0;
			yPosG = 0;
			trgr = 1;
			
		}
}



	if(opStat >= 6){
		
		deltaMillis4 = millis() - prevMillis4;
		
		for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

			// pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
			// Here we're using a moderately bright green color:
				pixels.setPixelColor(i, pixels.Color(255, 0, 0));

				pixels.show();   // Send the updated pixel colors to the hardware.
				}
				
				
		
		if(digitalRead(button2)==1 && deltaMillis4>=200){ // Is Button B pushed?
			Serial.print("B Button Push \n");		
		
			BPush = BPush+1;
			prevMillis4 = millis();
			

      if(BPush >=2 && deltaMillis4<=500){ //If B Button is pushed twice in 1.5 s, the operational status is changed
       Serial.print("Double Push B \n");
	   if(opStat>=1){
		   opStat = opStat_prev;
	   }
       BPush= 0;
     }
	 
	 for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

			// pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
			// Here we're using a moderately bright green color:
				pixels.setPixelColor(i, pixels.Color(0, 100, 0));

				pixels.show();   // Send the updated pixel colors to the hardware.
				}
		
		
		
	}

}
}
}


void loop(void) //Empty since threading is used
{

  
}

	

void readSensorData(){
	// Tell the bno what data we want
	  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  Heading = orientationData.orientation.x;
  
  
  if(initialized == false){ // Sets the initial heading for coordinate transformation
	  Serial.print("Initializing Heading \n");
	  Serial.print("Current Heading: ");
	 
	  Serial.print(Heading);
	  delay(500);
	  initHeading = Heading;
    Serial.print(initHeading);
	  delay(500);
	  signal(1);
	  initialized = true;
  }

  
   Heading = lowpassFiltering(0.1, Heading, Heading_Prio); // LPF for the heading to counter fast oszillation caused by walking
   Heading_Prio = Heading;
  
  if(linearAccelData.acceleration.y >= 4 && zc == 0){ //Detects if step startet
	step = step+1;  
	
	//position maths
	posx = posx + lHP*cos(DEG_2_RAD*Heading)*stepln;
	posy = posy + lHP*sin(DEG_2_RAD*Heading)*stepln;
	xPosG = cos(DEG_2_RAD*initHeading)*posx-sin(DEG_2_RAD*initHeading)*posy;
	yPosG = sin(DEG_2_RAD*initHeading)*posx+cos(DEG_2_RAD*initHeading)*posy;
	lHP = 1;
	//End Position maths
	
	zc = 1;

  }
  if(zc ==1 && samplingC <= 28){
	  samplingC = samplingC+1;
  }
  

  if(linearAccelData.acceleration.y <= -4 && zc == 1){ //detects if step ended

	zc = 0;
	samplingC = 0;
  }
  
  if(linearAccelData.acceleration.y >= 1){ //determines the time the last time the acceleration is greater than 1g
	  last_move = millis();
    noMove=0;
  }
  
  if(millis()-last_move >= 10000 && noMove==0){ //determines if no movement was detected for 10 s
	  noMove=1;
  }
  
 
  
  
  
  
  
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    char cstr[32];
    sprintf (cstr, "%04d,%02u,%03d,%05d,%05d", 1, opStat,step,int(xPosG*10),int(yPosG*10));
    radio.write(&cstr, sizeof(cstr)); // Sends data to rf transmitter
    i = i+1; 
    
    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }

}


void InitSteplen(){
	int step_prior;
	int prevMillis = millis(); 
	bool stop = false;
	int deltaMillis;
	
	while(stop==false){
		deltaMillis = millis() - prevMillis;
	
	if(deltaMillis >= 1000){
		if(step_prior == step && step_prior != 0){
			stop = true;
			stepln = (double)10/step_prior;
			Serial.print("Step length is:"); Serial.print(stepln);
			initSteplen = true;
			posx = 0;
			posy = 0;
			step = 0;
			delay(500);
			signal(1);
		}
	}
	
	if(stop == false){
		readSensorData();
		
		if(step >= step_prior+1){
			prevMillis = millis();
		}
		step_prior = step;
		
		
	}
	}
	
	
}


//MMI Functions

void signal(int status){
	if(mute == false){
			switch(status){
		case 1:
		 for(int i = 0;i<1;i++){
		  
		   tone(buzzer, 4186, // C
		   100); // half a second
		   
		 }
		  break;
		case 2:
		for(int i = 0;i<2;i++){
		
			tone(buzzer, 4186, // C
			100); // half a second
			delay(100);
			
			tone(buzzer, 3500, // C
			500); // half a second
			delay(500);
    
		}
		 break;
		case 3:
		
		
			tone(buzzer, 2100, // C
			100); // half a second
			delay(100);
    
		
		 break;
		  
		default:
		//nothing
		break;
			}
	}

}
	

double lowpassFiltering(double factor, double accF, double acc_prioF){
  // Lowpass Filter with exponential smoothing
  double res = factor*accF+(1-factor)*acc_prioF;

  return res;
  
}

void signalLightStatus(int status){
  
 
  switch(status) {
  case 1:
      for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(0, 180, 0));

    pixels.show();   // Send the updated pixel colors to the hardware.
    // Pause before next pass through loop
    }
    break;
  case 2:
      for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(252, 236, 3));

    pixels.show();   // Send the updated pixel colors to the hardware.
    // Pause before next pass through loop
   }
    break;
  case 3:
   for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(252, 3, 3));

    pixels.show();   // Send the updated pixel colors to the hardware.
    // Pause before next pass through loop
   }
	break;
  case 4:
    // Ampelfarbe Rot blinken schnell
  default:
    pixels.clear();
	 break;
}
}

void testHMI_features(){
	//Hinweis und Warnsignale
	signal(1);
	//signal(2);
	//signal(3);
	//Ampelfarben
	signalLightStatus(1);
  delay(1000);
	signalLightStatus(2);
  delay(1000);
	signalLightStatus(3);
  delay(1000);
	signalLightStatus(4);
}
