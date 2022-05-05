//Assigning ports
int nozzle=11;
//int pot=A0;
int bypass=10;

//PID constants
double kp = 50;
double ki = 0;
double kd = 0;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

//flow sensor 
byte statusLed    = 13;
byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin       = 2;

float calibrationFactor =7;
volatile byte pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;
//---------------------------
void setup() {
  Serial.begin(9600);
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH);  // We have an active-low LED attached
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;

attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
Serial.begin(9600);
Serial.println("Date & Time, Desired Flowrate , Actual Flowrate , DutyCyle");
pinMode(nozzle,OUTPUT);
pinMode(bypass,OUTPUT);
//pinMode(pot,INPUT);
}

void loop() {
 if((millis() - oldTime) > 1000)    // Only process counters once per second
  { 
    detachInterrupt(sensorInterrupt);
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    oldTime = millis();
    flowMilliLitres = (flowRate) * 1000;
    totalMilliLitres += flowMilliLitres;   
    unsigned int frac;  
    // Print the flow rate for this second in litres / minute
    //Serial.print("Flow rate: ");
    //Serial.print(int(flowRate));  // Print the integer part of the variable
    //Serial.print(".");             // Print the decimal point
    // Determine the fractional part. The 10 multiplier gives us 1 decimal place.
   //frac = (flowRate - int(flowRate)) * 10;
     // Serial.print(frac, DEC) ;      // Print the fractional part of the variable
    // Serial.println("L/min");
    // Print the number of litres flowed in this second
    //Serial.print("  Current Liquid Flowing: ");             // Output separator
    //Serial.print(flowMilliLitres);
    //Serial.print("mL/min");
    // Print the cumulative total of litres flowed since starting
    // Serial.print("  Output Liquid Quantity: ");             // Output separator
    //Serial.print(totalMilliLitres);
     //Serial.println("mL"); 
    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  }
//float val=analogRead(A0); float pwm=map(val,0,1023,0,4500);
int DesiredFlowRate=1000;
float error=DesiredFlowRate-flowMilliLitres;
error=abs(error);
output=computePID(error);
//Serial.print("Error= "); Serial.println(error);
//Serial.print("PID_output= ");Serial.println(output);
double c=map(output,0,10000,0,100);
//Serial.print("DutyCycle= ");Serial.println(dutyCycle);
double dutyCycle;
if (c<=10){
 dutyCycle=10;
 }
else if (c>=90){
 dutyCycle=90;
}
else
{dutyCycle=c;
}


  float frequency=10;
float  t=1/frequency*1000;
float  TH=t*dutyCycle/100;
  float TL=t-TH;
//Serial.print("TH= ");Serial.println(TH);
//Serial.print("TL= "); Serial.println(TL);
  digitalWrite(nozzle,HIGH);
  digitalWrite(bypass,HIGH);
  delay(TH);
  digitalWrite(nozzle,LOW);
  digitalWrite(bypass,HIGH);
  delay(TL);

Serial.print(","); Serial.print(DesiredFlowRate); Serial.print(","); Serial.print(flowMilliLitres); Serial.print(","); Serial.println(dutyCycle);

}
void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;
}

double computePID(double error){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation 
        error = error;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
        return out;                                        //have function return the PID output
}
