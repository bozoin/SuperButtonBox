#include <Wire.h>
#include <Encoder.h>
#include <Keypad.h>

/*
 * A20 et A21 réservé pour I2C
 * interrupt : 2, 3, 18, 19, 20, 21  priorité pour rotary encoder
 */

/*
********************************************************************************
**********************************GESTION LOCAL********************************
********************************************************************************
*/



/*
TODO


*/
#define VIDE 255

// DISPLAY
#define LOCAL_DISPLAY_NB 16
//const uint8_t localDisplay[LOCAL_DISPLAY_NB] = {22,23,24};
const uint8_t localDisplay[LOCAL_DISPLAY_NB] = {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37};

// ROTARY ENCODER
#define LOCAL_ROTARY_COUNT  6
// tableau des encoder rotatif (DT,CLK)
Encoder rotaryEncoder[LOCAL_ROTARY_COUNT] ={
  Encoder(2, 4),
  Encoder(3, 5),
  Encoder(18, 6),
  Encoder(19, 7),
  Encoder(20, 8),
  Encoder(21, 9)
};

// BUTTON 
#define LOCAL_SINGLE_BUTTON_NB 1   // single button / switch 2 / switch 3 COMPREND LES SWITCHS
const uint8_t localSingleButton[LOCAL_SINGLE_BUTTON_NB]={10};

// MATRICE DE BOUTON
//Définition de la matrice 4 x4 boutons
const byte ROWS1 = 4; //4 lignes
const byte COLS1 = 4; //4 colonnes
// Donner un code Ascii aux 16 touches, par exemple 0-F
char keys1[ROWS1][COLS1] = {
  {1,2,3,4},
  {5,6,7,8},
  {9,10,11,12},
  {13,14,15,16}
};
byte rowPins1[ROWS1] = {2, 3, 4, 5}; //les 4 lignes de la matrice sur les pins arduino digitales
byte colPins1[COLS1] = {6, 7, 8, 9}; //les 4 colonnes de la matrice

//Créer les objets matrice
#define LOCAL_MATRICE_COUNT  1
Keypad matrices[LOCAL_MATRICE_COUNT] = {
  Keypad(makeKeymap(keys1), rowPins1, colPins1, ROWS1, COLS1 )
};
const uint8_t localCountMatriceButton [LOCAL_MATRICE_COUNT]={ROWS1*COLS1};
const uint8_t localTotalMatriceButton = ROWS1*COLS1; // METTRE A JOUR EN AJOUTANT DES MATRICES

//SLIDER
#define LOCAL_SLIDER_NB 2
const int localSliders[LOCAL_SLIDER_NB] = {A0, A1};

//JOYSTICK
#define LOCAL_JOY_NB 0
const uint8_t localJoys[LOCAL_JOY_NB*2] = {};

#define I2C_SLAVE_ADDRESS 11 
#define T_CYCLE 15
#define BUTTON_COUNT 40 // (256 max)

// MASQUE
const uint8_t MASQUE_5b=0x1F;
#define DISPLAY_ON 0x88
#define DISPLAY_OFF 0x80

const uint8_t totalButton = LOCAL_SINGLE_BUTTON_NB + LOCAL_ROTARY_COUNT*2 + localTotalMatriceButton;

// Last state of the button
uint8_t lastButtonState[totalButton];
uint8_t lastSliderState[LOCAL_SLIDER_NB];

struct buttonUpdate
{
  uint8_t button;
  uint8_t state;
};

struct buttonUpdate buttonToSend;

void setup()
{
  Serial.begin(9600); //DEBUG

  initI2C();
  initDisplay();
  initButton();
  initRotary();
  initSlider();
}

void initI2C(){
  Wire.begin(I2C_SLAVE_ADDRESS);
  delay(1000);               
  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);
}

void initDisplay(){
  for (int disp = 0; disp<LOCAL_DISPLAY_NB;disp++ )
  {
    pinMode(localDisplay[disp],OUTPUT);
    digitalWrite(localDisplay[disp],LOW);
  }
}

void initRotary(){
  for(int i = 0;i<LOCAL_ROTARY_COUNT;i++){
      rotaryEncoder[i].write(0); // configure tous les rotary à 0
  }
}

void initButton(){
  for (uint8_t i =0; i<LOCAL_SINGLE_BUTTON_NB; i++)
  {
    pinMode(localSingleButton[i], INPUT_PULLUP);
  }
}

void initSlider(){
  for (uint8_t i =0; i<LOCAL_SLIDER_NB; i++)
  {
    pinMode(localSliders[i], INPUT);
  }
}

void loop(){
  
/*
  while (0 && Wire.available()) {
    uint8_t instruction = Wire.read();
    uint8_t com = Wire.read();
    if(instruction == DISPLAY_ON)
    {
          digitalWrite(localDisplay[com],HIGH);
    }
    else if(instruction == DISPLAY_OFF)
    {
          digitalWrite(localDisplay[com],LOW);
    }
  }  */
  if(buttonToSend.button==VIDE)
  {
    getLocalRotary();
  }
  if(buttonToSend.button==VIDE)
  {
    getLocalMatrice();
  }
  if(buttonToSend.button==VIDE)
  {
    getLocalButtons();
  }
  if(buttonToSend.button==VIDE)
  {
    getLocalAxis();
  }
  delay(T_CYCLE);  
}

void getLocalMatrice()
{

  int offsetButton= LOCAL_SINGLE_BUTTON_NB;

  for (int i=0;i<LOCAL_MATRICE_COUNT;i++)
  {
    char key = matrices[i].getKey();   //Surveiller les boutons
    if(key==NO_KEY)
    {
      for(int j=0; j<localCountMatriceButton[i];j++)
      {
        if (lastButtonState[offsetButton+j]==1 && buttonToSend.button==VIDE)
        {
          buttonToSend.button= offsetButton+j;
          buttonToSend.state=0;
          lastButtonState[offsetButton+j]=0;
        }
      }
    }
    else
    {
      Serial.println(key);  // DEBUG
      uint8_t button =key+offsetButton;
      if (lastButtonState[button]==0 && buttonToSend.button==VIDE)
      {
        buttonToSend.button= button;
        buttonToSend.state=1;
        lastButtonState[button]=1;
      }
    }
    offsetButton=offsetButton+localCountMatriceButton[i];
  }
}

void getLocalAxis(){

  uint8_t slider[LOCAL_SLIDER_NB];
  for(int i = 0; i<LOCAL_SLIDER_NB;i++){slider[i]=analogRead(localSliders[i])/4;}
  for(int i = 0; i<LOCAL_SLIDER_NB;i++)
  {
    if(lastSliderState[i]!=slider[i] && buttonToSend.button==VIDE)
    {
        buttonToSend.button= i+totalButton;
        buttonToSend.state=slider[i];
        lastSliderState[i]=slider[i];
    }  
  }
}

void getLocalButtons(){
  for (uint8_t i =0; i<LOCAL_SINGLE_BUTTON_NB; i++)
  {
    int currentButtonState = !digitalRead(localSingleButton[i]);
    if (currentButtonState != lastButtonState[i] && buttonToSend.button==VIDE)
    {
        buttonToSend.button= i;
        buttonToSend.state=currentButtonState;
        lastButtonState[i] = currentButtonState;
    }
  }
}

void getLocalRotary(){
  long rotaryRead;
  for(int i = 0; i<LOCAL_ROTARY_COUNT; i++){
    rotaryRead = rotaryEncoder[i].read();
      if (rotaryRead<0) {
        buttonToSend.button= LOCAL_SINGLE_BUTTON_NB+localTotalMatriceButton+i*2;
        buttonToSend.state=1;
        rotaryEncoder[i].write(0);
      }
      else if (rotaryRead>0) {
        buttonToSend.button= LOCAL_SINGLE_BUTTON_NB+localTotalMatriceButton+i*2+1;
        buttonToSend.state=1;
        rotaryEncoder[i].write(0);
      }
  }
}

void requestEvents()
{
  /*
  uint8_t slider[SLIDER_COUNT];
  for(int i = 0; i<SLIDER_COUNT;i++){slider[i]=analogRead(sliderTest)/4;}
  for(int i = 0; i<SLIDER_COUNT;i++){Wire.write(slider[i]);}
  */
  Wire.write(buttonToSend.button);
  Wire.write(buttonToSend.state);
  buttonToSend.button=VIDE;
}

void receiveEvents(int numBytes)
{  
  int n[numBytes];
  for (int i=0; i<numBytes; i++)
  {
      n[i]= Wire.read();
      Serial.println((int)n[i]);
  }
  for (int i=0; i<numBytes; i=i+2)
  {
    if(n[i] == DISPLAY_ON)
    {
          digitalWrite(localDisplay[n[i+1]],HIGH);
    }
    
    else if(n[i] == DISPLAY_OFF)
    {
          digitalWrite(localDisplay[n[i+1]],LOW);
    }
  }
}