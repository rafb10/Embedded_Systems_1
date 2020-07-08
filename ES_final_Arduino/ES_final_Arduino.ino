#include <Wire.h>
#include <SPI.h>

uint8_t i2c_out[2];
uint16_t potentiometer_from_STM;

volatile byte spi_from_STM;
volatile byte spi_received_tab[2];
volatile boolean spi_received = false;
volatile byte cnt = 0;

int photoresistor_pin = A0; 
uint16_t photoresistor_val;


// i2c - send photoresistor measurement
// function that is executed whenever data is requested by i2c master STM32
void requestEvent() {
  photoresistor_val = analogRead(photoresistor_pin);
  i2c_out[1] = (photoresistor_val >> 8) & 0xFF;
  i2c_out[0] = photoresistor_val & 0xFF;
  Wire.write(i2c_out[0]); 
  Wire.write(i2c_out[1]);
}

// spi receive
ISR (SPI_STC_vect)                    
{
//  spi_from_STM = SPDR;         // Value received from master if store in variable slavereceived
//  spi_received = true;          // this flag is used in loop() 
    spi_received_tab[cnt] = SPDR;
    spi_received = true;  //set the flag to know that we get new value from spi Master 
    if(cnt==0)
      cnt++;
    else 
      cnt = 0;
}

void setup() {
  //i2c setup
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  pinMode(MOSI,INPUT); 

  //spi setup
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPCR |= _BV(SPE);     //Turn on SPI in Slave Mode
  SPCR |= _BV(SPIE);
  spi_received = false;

  //serial setup
  Serial.begin(9600); 
}

void loop() {
  if(spi_received == true){
    if(cnt == 0){
      potentiometer_from_STM = spi_received_tab[1] << 8 | spi_received_tab[0];
      Serial.print("Potentiometer: ");
      Serial.print(potentiometer_from_STM); 
      Serial.print(" ");
    }
    spi_received = false; // clear the flag
  }
}




