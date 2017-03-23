
 *******************************************************************************************/
 
/* 
  Layout for the 3x3x3 cube
  Row 1 = Top layer, 9 LEDs
  Row 2 = Middle Layer, 9 LEDs
  Row 3 = Bottom layer, 9 LEDs
  
  Col 1 = Front Left 3 LEDS 1 top, 2 middle, 3 bottom
  Col 2 = Front Middle 3 LEDS 4 top, 5 middle, 6 bottom
  Col 3 = Front Right 3 LEDS 7 top, 8 middle, 9 bottom
  
  ****need to complete this documentation 
  Col 4 = Middle Right 3 LEDS - 8 top, 9 middle, 10 bottom
  Col 3 = Front Right 3 LEDS 7 top, 8 middle, 9 bottom
  Col 3 = Front Right 3 LEDS 7 top, 8 middle, 9 bottom
  Col 3 = Front Right 3 LEDS 7 top, 8 middle, 9 bottom
  Col 3 = Front Right 3 LEDS 7 top, 8 middle, 9 bottom
  Col 3 = Front Right 3 LEDS 7 top, 8 middle, 9 bottom
*/
 
 
// Row control
int ledRow1 = 13;  //Row 1
int ledRow2 = 12;  //Row 2
int ledRow3 = 11;  //Row 3


//Cols 1,2,3 + Rows can control LEDS 1-9
int ledCol9 = 10;
int ledCol8 = 9;
int ledCol7 = 8;

//Cols 4,5,6 + Rows can control LEDS 10-12
int ledCol6 = 7;
int ledCol5 = 6;
int ledCol4 = 5;

//Cols 4,5,6 + Rows can control LEDS 10-12
int ledCol3 = 4;
int ledCol2 = 3;
int ledCol1 = 2;


//define the LED cube size
int rowmax = 3;
int colmax = 9;
//making an array
int ledCol[9];
int ledRow[3];

// the setup routine runs once when you press reset:
void setup() {                
  
  //used for random value
  //Serial.begin(9600);
  // initialize the digital pin as an output.
  
  //3 rows
  pinMode(ledRow1, OUTPUT); //Top
  pinMode(ledRow2, OUTPUT); //Middle
  pinMode(ledRow3, OUTPUT); //Bottom
  
  //9 Columns
  pinMode(ledCol1, OUTPUT); //Left
  pinMode(ledCol2, OUTPUT); //Middle
  pinMode(ledCol3, OUTPUT); //Right
  pinMode(ledCol4, OUTPUT); //Left
  pinMode(ledCol5, OUTPUT); //Middle
  pinMode(ledCol6, OUTPUT); //Right
  pinMode(ledCol7, OUTPUT); //Left
  pinMode(ledCol8, OUTPUT); //Middle
  pinMode(ledCol9, OUTPUT); //Right
  
  
  //fill array
  ledCol[0] = ledCol1;
  ledCol[1] = ledCol2;
  ledCol[2] = ledCol3;
  ledCol[3] = ledCol4;
  ledCol[4] = ledCol5;
  ledCol[5] = ledCol6;
  ledCol[6] = ledCol7;
  ledCol[7] = ledCol8;
  ledCol[8] = ledCol9;
  
  ledRow[0] = ledRow1;
  ledRow[1] = ledRow2;
  ledRow[2] = ledRow3;

//  testAllLEDs(); //checks all the LEDS make sure they can all light. 
}

// the loop routine runs over and over again forever:
void loop() {


upper_row();
delay(1000);
middle_row();
delay(1000);
lower_row();
delay(1000);
drops();
delay(1000);
all_row();
delay(1000);
all_col();
delay(1500);
led_cube();
}




void upper_row()
{
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],HIGH);

digitalWrite(ledCol[0],HIGH);
delay(250);
digitalWrite(ledCol[0],LOW); 
digitalWrite(ledCol[1],HIGH);
delay(250);
digitalWrite(ledCol[1],LOW);
digitalWrite(ledCol[2],HIGH);
delay(250);
digitalWrite(ledCol[2],LOW);
digitalWrite(ledCol[3],HIGH);
delay(250);
digitalWrite(ledCol[3],LOW);
digitalWrite(ledCol[4],HIGH);
delay(250);
digitalWrite(ledCol[4],LOW);
digitalWrite(ledCol[5],HIGH);
delay(250);
digitalWrite(ledCol[5],LOW);
digitalWrite(ledCol[6],HIGH);
delay(250);
digitalWrite(ledCol[6],LOW);
digitalWrite(ledCol[7],HIGH);
delay(250);
digitalWrite(ledCol[7],LOW);

  }

void middle_row()
{
  digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],HIGH);

digitalWrite(ledCol[0],HIGH);
delay(250);
digitalWrite(ledCol[0],LOW); 
digitalWrite(ledCol[1],HIGH);
delay(250);
digitalWrite(ledCol[1],LOW);
digitalWrite(ledCol[2],HIGH);
delay(250);
digitalWrite(ledCol[2],LOW);
digitalWrite(ledCol[3],HIGH);
delay(250);
digitalWrite(ledCol[3],LOW);
digitalWrite(ledCol[4],HIGH);
delay(250);
digitalWrite(ledCol[4],LOW);
digitalWrite(ledCol[5],HIGH);
delay(250);
digitalWrite(ledCol[5],LOW);
digitalWrite(ledCol[6],HIGH);
delay(250);
digitalWrite(ledCol[6],LOW);
digitalWrite(ledCol[7],HIGH);
delay(250);
digitalWrite(ledCol[7],LOW);

  }
void lower_row()
{
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],LOW);

digitalWrite(ledCol[0],HIGH);
delay(250);
digitalWrite(ledCol[0],LOW); 
digitalWrite(ledCol[1],HIGH);
delay(250);
digitalWrite(ledCol[1],LOW);
digitalWrite(ledCol[2],HIGH);
delay(250);
digitalWrite(ledCol[2],LOW);
digitalWrite(ledCol[3],HIGH);
delay(250);
digitalWrite(ledCol[3],LOW);
digitalWrite(ledCol[4],HIGH);
delay(250);
digitalWrite(ledCol[4],LOW);
digitalWrite(ledCol[5],HIGH);
delay(250);
digitalWrite(ledCol[5],LOW);
digitalWrite(ledCol[6],HIGH);
delay(250);
digitalWrite(ledCol[6],LOW);
digitalWrite(ledCol[7],HIGH);
delay(250);
digitalWrite(ledCol[7],LOW);

}


void drops()
{
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[0],HIGH);
delay(750);  
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[0],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[0],HIGH);
delay(1000);
digitalWrite(ledCol[0],LOW);
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[4],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[4],HIGH);
delay(750);  
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[4],HIGH);
delay(1000);
digitalWrite(ledCol[4],LOW);
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[6],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[6],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[6],HIGH);
delay(1000);
digitalWrite(ledCol[6],LOW);
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[2],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[2],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[2],HIGH);
delay(1000);
digitalWrite(ledCol[2],LOW);
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[3],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[3],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[3],HIGH);
delay(1000);  
digitalWrite(ledCol[3],LOW);
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[7],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],HIGH);
digitalWrite(ledCol[7],HIGH);
delay(750);
digitalWrite(ledRow[0],HIGH);
digitalWrite(ledRow[1],HIGH);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[7],HIGH);
delay(1000);  
digitalWrite(ledCol[7],LOW);
  }
void all_row()

{
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[0],HIGH);
delay(500);
digitalWrite(ledCol[0],LOW);
digitalWrite(ledCol[1],HIGH);
delay(500);
digitalWrite(ledCol[1],LOW);
digitalWrite(ledCol[2],HIGH);
delay(500);
digitalWrite(ledCol[2],LOW);
digitalWrite(ledCol[3],HIGH);
delay(500);
digitalWrite(ledCol[3],LOW);
digitalWrite(ledCol[4],HIGH);
delay(500);
digitalWrite(ledCol[4],LOW);
digitalWrite(ledCol[5],HIGH);
delay(500);
digitalWrite(ledCol[5],LOW);
digitalWrite(ledCol[6],HIGH);
delay(500);
digitalWrite(ledCol[6],LOW);
digitalWrite(ledCol[7],HIGH);
delay(500);    
digitalWrite(ledCol[7],LOW);  
delay(1500);  
  
    }
void all_col()
{
  digitalWrite(ledRow[0],LOW);
  digitalWrite(ledRow[1],HIGH);
  digitalWrite(ledRow[2],HIGH);
  digitalWrite(ledCol[0],HIGH);
  digitalWrite(ledCol[1],HIGH);
  digitalWrite(ledCol[2],HIGH);
  digitalWrite(ledCol[3],HIGH);
  digitalWrite(ledCol[4],HIGH);
  digitalWrite(ledCol[5],HIGH);
  digitalWrite(ledCol[6],HIGH);
  digitalWrite(ledCol[7],HIGH);
  digitalWrite(ledCol[8],HIGH);
  delay(750);
  digitalWrite(ledRow[0],HIGH);
  digitalWrite(ledRow[1],LOW);
  digitalWrite(ledRow[2],HIGH);
  delay(750);
  digitalWrite(ledRow[0],HIGH);
  digitalWrite(ledRow[1],HIGH);
  digitalWrite(ledRow[2],LOW);
  delay(750);
  digitalWrite(ledRow[0],LOW);
  digitalWrite(ledRow[1],HIGH);
  digitalWrite(ledRow[2],HIGH);
  delay(500);
  digitalWrite(ledRow[0],HIGH);
  digitalWrite(ledRow[1],LOW);
  digitalWrite(ledRow[2],HIGH);
  delay(500);
  digitalWrite(ledRow[0],HIGH);
  digitalWrite(ledRow[1],HIGH);
  digitalWrite(ledRow[2],LOW);
  delay(500);
  digitalWrite(ledRow[0],LOW);
  digitalWrite(ledRow[1],HIGH);
  digitalWrite(ledRow[2],HIGH);
  delay(250);
  digitalWrite(ledRow[0],HIGH);
  digitalWrite(ledRow[1],LOW);
  digitalWrite(ledRow[2],HIGH);
  delay(250);
  digitalWrite(ledRow[0],HIGH);
  digitalWrite(ledRow[1],HIGH);
  digitalWrite(ledRow[2],LOW);
  delay(250);
  }    

void led_cube()
{
  
digitalWrite(ledRow[0],LOW);
digitalWrite(ledRow[1],LOW);
digitalWrite(ledRow[2],LOW);
digitalWrite(ledCol[0],HIGH);
digitalWrite(ledCol[1],HIGH);
digitalWrite(ledCol[2],HIGH);
digitalWrite(ledCol[3],HIGH);
digitalWrite(ledCol[4],HIGH);
digitalWrite(ledCol[5],HIGH);
digitalWrite(ledCol[6],HIGH);
digitalWrite(ledCol[7],HIGH);
  }  
