#include <SmoothThermistor.h>


//includes


#include <BME280I2C.h>
BME280I2C bme;  

bool metric = true;

/*
 * HVAC Control Unit (hvac_cu)
 * Peter Cairns
 * June 2017
  Connecting the BME280 Sensor:
  Sensor              ->  Board
  -----------------------------
  Vin (Voltage In)    ->  3.3V
  Gnd (Ground)        ->  Gnd
  SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
  SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro
*/

SmoothThermistor smoothThermistor1(A0,ADC_SIZE_10_BIT,100000,100000,3950,25,10);
SmoothThermistor smoothThermistor2(A1,ADC_SIZE_10_BIT,100000,100000,3950,25,10);
SmoothThermistor smoothThermistor3(A2,ADC_SIZE_10_BIT,100000,100000,3950,25,10);

// set parameters
float min_house_temp=21.0;    //minimum house temperature, winter
float max_house_temp=25.0;    //maximum house temperature, summer
float fr_high=200;      //high VMC flowrate [m3/hr]
float fr_low=150;       //low VMC flowrate [m3/hr]
float cp=1.005;         //specific heat air [kJ/kg.K]
float rho=1.2;          //air density [kg/m3]

float q;             //heat loss [kW]  
float in_fr_en;      //fresh inlet energy [kW/m3]  
float in_wa_en;      //waste inlet energy [kW/m3]
float ex_fr_en;      //fresh exhaust energy [kW/m3]
float efficiency;    //heat exchanger efficiency

float delta_av=0;

int winter_mode=0;   //disables bypass & cooling during winter months, default disabled
int cool_mode=1;     //cooling mode (default enabled =1)
int heat_mode=1;     //heating mode (default enabled =1)
int pin_heat=13;     //gas heater relay control pin
int pin_cool=12;     //cooler compressor relay control pin
int pin_byp=11;      //vmc bypass relay control pin
int pin_spd=10;      //vmc fan speed relay control pin
int pin_spk=4;       //internal speaker pin
int counter=0;


// set variables
float in_fr_temp;   //fresh inlet temp [C]
float in_fr_humd;   //fresh inlet relative humidity [%]
float in_fr_pres;   //fresh inlet pressure [kPa]
float in_wa_temp;   //waste inlet temp [C]
float ex_fr_temp;   //fresh exhaust temp [C]
float ex_wa_temp;   //waste exhaust temp [C]
float house_inlet;  // fresh air feed to house (post HE) [C]
 
void setup() {



  
   // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.println("VMC Control Unit v1.0");
  Serial.println("=====================");

  // start BME280 functionality
  bme.begin();

  // startup tone
  tone(pin_spk,3000,200);

  // set our pin modes
  pinMode(pin_heat, OUTPUT);
  pinMode(pin_cool, OUTPUT);
  pinMode(pin_byp, OUTPUT);
  pinMode(pin_spd, OUTPUT);

}

void loop() {

/*
 *  capture the sensor data
 *  -----------------------
 */
  
  bme.read(in_fr_pres,in_fr_temp,in_fr_humd);
  in_wa_temp = smoothThermistor1.temperature();
  ex_fr_temp = smoothThermistor2.temperature();
  ex_wa_temp = smoothThermistor3.temperature();
  

/*
 *  ventilation bypass activation
 *  -----------------------------
 */

  // choose correct temp sensor for fresh air
  if (digitalRead(pin_byp))
    {
      house_inlet = ex_fr_temp;
    }
  else
    {
      house_inlet = in_fr_temp;
    }

  // pull in outside air if cooler than inside air, winter mode off, house not too cold
  if (winter_mode == 0 && in_wa_temp > min_house_temp && house_inlet < (in_wa_temp-1) )
    {
       digitalWrite(pin_byp, HIGH);
    }
  else
    {
      digitalWrite(pin_byp, LOW);
    }


/*
 *  cooling activation
 *  -------------------
 */

  // turn on cooling compressor if house too hot, bypass disabled, winter mode off
  if (winter_mode == 0 && !digitalRead(pin_byp) && in_wa_temp > max_house_temp)
    {
       digitalWrite(pin_cool, HIGH);
       digitalWrite(pin_spd, HIGH);
    }
  else
    {
      digitalWrite(pin_cool, LOW);
      digitalWrite(pin_spd, LOW);
    } 


/*
 *  central heating activation
 *  --------------------------
 */

  // TODO insert PID algorithm


/*  
 *  calculate the heat exchanger efficency
 *  --------------------------------------
 */

  in_fr_en = cp*rho*(in_fr_temp+273);
  in_wa_en = cp*rho*(in_wa_temp+273);
  ex_fr_en = cp*rho*(ex_fr_temp+273);
  efficiency = (ex_fr_en-in_fr_en)/(in_wa_en-in_fr_en);
  
//  if (in_fr_temp > in_wa_temp)
//  {
//    efficiency = 1-efficiency; 
//  }

/*
 *  output sensor data to the serial port
 *  --------------------------------------
 */

//  Serial.print("Fresh Inlet Temp: " );
//  Serial.println(in_fr_temp);
//  Serial.print("Waste Inlet Temp: " );
//  Serial.println(in_wa_temp);  
//  Serial.print("Fresh Outlet Temp: " );
//  Serial.println(ex_fr_temp);  
//  Serial.print("Waste Outlet Temp: " );
//  Serial.println(ex_wa_temp);  
//  Serial.println("");  
//  
//  Serial.print("Fresh Inlet Pressure: " );
//  Serial.print(in_fr_pres/1000);    
//  Serial.println("kPa");  
//  Serial.print("Fresh Inlet Humidity: " );
//  Serial.print(in_fr_humd);  
//  Serial.println("%");
//  Serial.println("");   
//    
//  Serial.print("Heat Exchanger Efficiency: ");
//  Serial.print(efficiency*100);
//  Serial.println("%");
//  Serial.print("delta temp (instantaneous): ");
//  Serial.println(in_fr_temp - in_wa_temp);
//  Serial.println("");
//
//  Serial.print("Bypass Mode: ");
//  Serial.println(digitalRead(pin_byp));
//
//  Serial.print("Cooling Mode: ");
//  Serial.println(digitalRead(pin_cool)); 

  //raw sensor data to serial lines
//  Serial.print(in_fr_temp);
//  Serial.print(",");
  Serial.print(in_wa_temp);
  Serial.print(",");
  Serial.print(ex_fr_temp);
  Serial.print(",");
  Serial.println(ex_wa_temp);

  
  
 // delay(3000);  // delete this & replace with sensible delay

  
  delta_av = delta_av + (in_fr_temp - in_wa_temp);
  counter = counter + 1;
//
//  if (counter = 20)
//    {
//      counter = 1;
//      delta_av = 0;
//    }

//  Serial.print("counter: ");
//  Serial.println(counter);    
//  Serial.print("running average: ");
//  Serial.println(delta_av/counter);    

}




