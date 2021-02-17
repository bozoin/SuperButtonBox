//free
#include <Adafruit_SSD1306.h>
#include <Joystick.h>
#include <Wire.h>
//#include <Encoder.h>

/*
 * Encoder rotaryEncoder(DT, CLK);
 * Interrupt : 0, 1, 2, 3, 7 priorité pour rotary encoder
 * A2 et A3 réservé pour I2C
 */

/*
********************************************************************************
**********************************GESTION GLOBAL********************************
********************************************************************************
*/

// écrans display
#define DISPLAYCOUNT 32
#define DISPLAYCONFCOUNT 3
const String displayText[DISPLAYCONFCOUNT][DISPLAYCOUNT]={
  {"M2K"    ,"1 2"    ,"1Button3"    ,"Button4"    ,"Button5"    ,"Button6"    ,"Button7"    ,"Button8"    ,"Button9"    ,"Button10"    ,"Button11"    ,"Button12"    ,"Button13"    ,"Button14"    ,"Button15"    ,"Button16"    ,"Button17"    ,"Button18"    ,"Button19"    ,"Button20"    ,"Button21"    ,"Button22"     ,"Button23"    ,"Button24"    ,"Button25"    ,"Button26"    ,"Button27"    ,"Button28"    ,"Button29"    ,"Button30"    ,"Button31"    ,"Button32"},
  {"IL2"    ,"2 2"    ,"2Button3"    ,"Button4"    ,"Button5"    ,"Button6"    ,"Button7"    ,"Button8"    ,"Button9"    ,"Button10"    ,"Button11"    ,"Button12"    ,"Button13"    ,"Button14"    ,"Button15"    ,"Button16"    ,"Button17"    ,"Button18"    ,"Button19"    ,"Button20"    ,"Button21"    ,"Button22"     ,"Button23"    ,"Button24"    ,"Button25"    ,"Button26"    ,"Button27"    ,"Button28"    ,"Button29"    ,"Button30"    ,"Button31"    ,"Button32"},
  {"Button1"    ,"3 2"    ,"3Button3"    ,"Button4"    ,"Button5"    ,"Button6"    ,"Button7"    ,"Button8"    ,"Button9"    ,"Button10"    ,"Button11"    ,"Button12"    ,"Button13"    ,"Button14"    ,"Button15"    ,"Button16"    ,"Button17"    ,"Button18"    ,"Button19"    ,"Button20"    ,"Button21"    ,"Button22"     ,"Button23"    ,"Button24"    ,"Button25"    ,"Button26"    ,"Button27"    ,"Button28"    ,"Button29"    ,"Button30"    ,"Button31"    ,"Button32"}
};
uint8_t current_display_conf=0;

// Screen
#define OLED_RESET  -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);

// Constantes global
#define BUTTON_MAX 254 // (254 max)
#define T_CYCLE 15
#define VIDE 255
#define DISPLAY_I2C 0x3c
#define MAX_AXIS 255

//TYPES
#define TYPE_BUTTON 1
#define TYPE_JOY 2
#define TYPE_SLIDER 3
#define TYPE_ROTARY 4
#define TYPE_SWITCH2 5
#define TYPE_SWITCH3 6

#define CONF_BOARDS 7

uint8_t BOARD_COUNT;
uint8_t * rotaryCount;
uint8_t * buttonCount;
uint8_t * sliderCount;
uint8_t * joystickCount;
uint8_t * displayCount;
uint8_t * switch2Count;
uint8_t * switch3Count;

uint8_t * i2c_addr;

// map analog
#define maxRotaryPot 63
#define minRotaryPot 0
#define MAX_ROTARY_BUTTON 10

Joystick_ * pads;

// MASQUE
//const uint8_t MASQUE_5b=0x1F;  //Plus utilisé
#define EXTERNAL_DISPLAY_ON 0x88
#define EXTERNAL_DISPLAY_OFF 0x80

// Last state of the button // TODO pas besoin de last state pour les boards ? mis à jour en fonction de ce qu'on reçoit
uint8_t ** lastButtonState;// un tableau par board ,index : numéro du board & index : numéro du bouton
uint8_t ** lastSliderState; 
uint8_t ** lastRotaryState;
uint8_t ** lastRotaryAxeState;
uint8_t ** lastSwitch2State;
uint8_t ** lastSwitch3State;
uint8_t ** lastJoyState;
uint8_t ** lastRotaryButState;

// contenu des écrans
String * displayTxt;
uint8_t displayTotal=0;

// Bouton à remise à zero
uint8_t doRaz=0;
uint8_t boardRaz;
uint8_t buttonRaz;

/*
********************************************************************************
**********************************GESTION LOCAL********************************
********************************************************************************
*/
// DISPLAY
#define LOCAL_DISPLAY_NB 1
const uint8_t localDisplay[LOCAL_DISPLAY_NB] = {5};

// BUTTON
#define LOCAL_SINGLE_BUTTON_NB 5

#define LOCAL_NEXT_DISPLAY_IN 4
#define LOCAL_ROT_PULSE_IN 5
#define LOCAL_ROT_AXE_IN 6
#define LOCAL_ROT_KNOB_IN 7
#define LOCAL_SWITCH_PULSE_IN 8

#define NEXT_DISPLAY 1
#define ROT_PULSE 2
#define ROT_AXE 3
#define ROT_KNOB 4
#define SWITCH_PULSE 5
const uint8_t localSingleButton[LOCAL_SINGLE_BUTTON_NB]={LOCAL_NEXT_DISPLAY_IN,LOCAL_ROT_PULSE_IN,LOCAL_ROT_AXE_IN,LOCAL_ROT_KNOB_IN,LOCAL_SWITCH_PULSE_IN};
uint8_t lastLocalButtonState[LOCAL_SINGLE_BUTTON_NB];


void setup() {
  // Attend l'initialisation des autres boards
  //Serial.begin(9600); DEBUG
  delay(2000);
  // init i2c
  Wire.begin(); 
  // find devices
  i2cScann();
  // initialise les tableaux en fonction du nombre de devices
  initTab();
  // récupère les conf (nb de boutons et d'axe de chaque boards)
  getBoardsConf();
  // initialise les variables en fonctions des conf des boards
  initVar();

  for (int i=0; i<LOCAL_SINGLE_BUTTON_NB;i++)
  {
    lastLocalButtonState[i]=0;
  } 

  // Init Button Pins TESTS
  initButton();

  // Init pads 
  for (int i = 0; i <BOARD_COUNT;i++)
  {
    pads[i].begin();
  }
  
  setAxisRange();

  // Init screens
  initDisplay();
}

void initTab(){
  pads = (Joystick_*)malloc(BOARD_COUNT*sizeof(Joystick_));
  for (int i =0; i<BOARD_COUNT;i++)
  {
    pads[i]=Joystick_(i2c_addr[i], JOYSTICK_TYPE_JOYSTICK, BUTTON_MAX, 0, true, true, true, true, true, true, true, true, true, true, true);

  }
  rotaryCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  buttonCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  sliderCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  joystickCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  displayCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  switch2Count= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  switch3Count= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));

  lastButtonState= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  lastSliderState= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  lastRotaryState= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  lastRotaryAxeState= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  lastRotaryButState= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  lastJoyState= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  lastSwitch2State= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  lastSwitch3State= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
}

void initVar(){
  displayTotal=LOCAL_DISPLAY_NB;
  // Init & malloc tab
  for (int i = 0; i<BOARD_COUNT;i++){
    displayTotal+=displayCount[i];
    lastSliderState[i]=(uint8_t*)malloc((sliderCount[i])*sizeof(uint8_t));
    for (int j = 0; i<sliderCount[i]; j++){
      lastButtonState[i][j]=0;
    }

    lastJoyState[i]=(uint8_t*)malloc((joystickCount[i]*2)*sizeof(uint8_t));
    for (int j = 0; i<joystickCount[i]*2; j++){
      lastJoyState[i][j]=0;
    }

    lastSwitch2State[i]=(uint8_t*)malloc((switch2Count[i])*sizeof(uint8_t)); // 0 ou 1
    for (int j = 0; i<switch2Count[i]; j++){
      lastSwitch2State[i][j]=0;
    }  

    lastSwitch3State[i]=(uint8_t*)malloc((switch3Count[i])*sizeof(uint8_t));// 0 ou 1 ou 2
    for (int j = 0; i<switch3Count[i]; j++){
      lastSwitch3State[i][j]=0;
    }  

    lastButtonState[i]=(uint8_t*)malloc((buttonCount[i])*sizeof(uint8_t));
    for (int j = 0; i<buttonCount[i]; j++){
      lastButtonState[i][j]=0;
    }  
    
    lastRotaryState[i]=(uint8_t*)malloc((rotaryCount[i])*sizeof(uint8_t));
    for (int j = 0; i<rotaryCount[i]; j++){
      lastRotaryState[i][j]=0;
    }  

    lastRotaryAxeState[i]=(uint8_t*)malloc((rotaryCount[i])*sizeof(uint8_t));
    for (int j = 0; i<rotaryCount[i]; j++){
      lastRotaryAxeState[i][j]=0;
    }    

    lastRotaryButState[i]=(uint8_t*)malloc((rotaryCount[i]*MAX_ROTARY_BUTTON)*sizeof(uint8_t));
    for (int j = 0; i<rotaryCount[i]*MAX_ROTARY_BUTTON; j++){
      lastRotaryButState[i][j]=0;
    }    
  }
  displayTxt=(String*)malloc(displayTotal*sizeof(String));
}

void i2cScann(){
  byte error, address;
  uint8_t nDevices = 0;
  uint8_t devices[127];
  
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error==0 && address!=DISPLAY_I2C)
    {
      nDevices++;
      devices[nDevices]=address;
    }
  }
  BOARD_COUNT=nDevices;
  i2c_addr=(uint8_t*)malloc((BOARD_COUNT)*sizeof(uint8_t));
  for (uint8_t i=0;i<nDevices;i++)
  {
    i2c_addr[i]=devices[i];
  }
}

void getBoardsConf(){
  for (uint8_t j=0;j<BOARD_COUNT;j++)
  {
    Wire.beginTransmission(i2c_addr[j]); 
    uint8_t buff=VIDE;
    uint8_t nbRequete=CONF_BOARDS;
    Wire.write( &buff, 1);          
    Wire.endTransmission();

    Wire.requestFrom(i2c_addr[j], nbRequete);

    displayCount[j]=Wire.read();
    buttonCount[j]=Wire.read();
    switch2Count[j]=Wire.read();
    switch3Count[j]=Wire.read();
    rotaryCount[j]=Wire.read();
    sliderCount[j]=Wire.read();
    joystickCount[j]=Wire.read();
//nb display | nb button | nb switch 2 | nb switch 3 | nb rotary | nb axes | nb joy
    delay(100);
  }

}

void initDisplay(){
  // initialisation des display local des pins en output et en écoute (high)
  uint8_t disp = 0;
  for (disp = 0; disp<LOCAL_DISPLAY_NB;disp++ )
  {
    pinMode(localDisplay[disp],OUTPUT);
    digitalWrite(localDisplay[disp],HIGH);
  }

  // initialisation des display externes des pins en output et en écoute (high)
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {
        uint8_t buff[2]={EXTERNAL_DISPLAY_ON,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 


       // sendI2CMsg(msg, i2c_addr[i]); //mets le premier bite de j à 1 pour mettre j en high
      }
  }

  delay(1000); // on attend que tous les écran sont prêts

  display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C);
  delay(100);
        display.clearDisplay();
        display.setTextSize(2); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("test"));
        display.display();

  for (int i = 0; i<LOCAL_DISPLAY_NB;i++ )
  {
    digitalWrite(localDisplay[i],LOW);
  }

  // mets les display externe en low
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {
        uint8_t buff[2]={EXTERNAL_DISPLAY_OFF,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 
      }
  }

  //  rempli le tableau de base des écrans
  for (int i = 0; i<displayTotal; i++)
  {
    displayTxt[i]=i;
  }

  delay(1000); // on attend que tous les écran sont prêts

  refreshScreen();
}

void initButton(){
  for (uint8_t i =0; i<LOCAL_SINGLE_BUTTON_NB; i++)
  {
    pinMode(localSingleButton[i], INPUT_PULLUP);
  }
}

void setAxisRange()
{
  for (uint8_t joyNum=0; joyNum<BOARD_COUNT; joyNum++)
  {
    pads[joyNum].setXAxisRange(0,255);
    pads[joyNum].setYAxisRange(0,255);
    pads[joyNum].setZAxisRange(0,255);
    pads[joyNum].setRxAxisRange(0,255);
    pads[joyNum].setRyAxisRange(0,255);
    pads[joyNum].setRzAxisRange(0,255);
    pads[joyNum].setRudderRange(0,255);
    pads[joyNum].setThrottleRange(0,255);
    pads[joyNum].setAcceleratorRange(0,255);
    pads[joyNum].setBrakeRange(0,255);
    pads[joyNum].setSteeringRange(0,255);
    for(uint8_t i = 0; i<11;i++)
    {
        updateAxis(joyNum,i,0);
    }

  }
}

/*
***********************************
LOOOP
***********************************
*/


void loop() {
  // Read local button values
  getLocalButton();
  
  requestButtonUpdate();
  delay(T_CYCLE);

  if (doRaz==1)
  {
    pads[boardRaz].setButton(buttonRaz, 0);
    doRaz=0;
  }
}

void confScreen(){
  // initialisation des display local des pins en output et en écoute (high)
  uint8_t confN = current_display_conf;
  int disp = 0;
  while (disp<LOCAL_DISPLAY_NB)
  {
      digitalWrite(localDisplay[disp],HIGH);
      display.clearDisplay();
      if (displayText[confN][disp].length()<4)
      {
        display.setTextSize(5);
      }
      else if (displayText[confN][disp].length()<7)
      {
        display.setTextSize(3);
      }
      else
      {
        display.setTextSize(1);
      }
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      //display.println(F(displayText[confN][disp]));
      display.println(displayText[confN][disp]);
      display.display();
      digitalWrite(localDisplay[disp],LOW);
      delay(100);
      disp++;
  }
  // initialisation des display externes des pins en output et en écoute (high)
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {
        uint8_t buff[2]={EXTERNAL_DISPLAY_ON,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 

        delay(100);

        display.clearDisplay();
        if (displayText[confN][disp].length()<4)
        {
          display.setTextSize(5);
        }
        else if (displayText[confN][disp].length()<7)
        {
          display.setTextSize(3);
        }
        else
        {
          display.setTextSize(1);
        }
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        //display.println(F(displayText[confN][disp]));
        display.println(displayText[confN][disp]);
        display.display();      // Show initial text
        delay(100);
        uint8_t buff2[2]={EXTERNAL_DISPLAY_OFF,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff2,2);
        Wire.endTransmission(); 
        disp++;
      }
  }
}

void refreshScreen(){
  // initialisation des display local des pins en output et en écoute (high)

  int disp = 0;
  while (disp<LOCAL_DISPLAY_NB)
  {
      digitalWrite(localDisplay[disp],HIGH);
      display.clearDisplay();
      display.setTextSize(5); // Draw 2X-scale text
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      //display.println(F("test"));
      display.println(disp);
      display.display();      // Show initial text
      digitalWrite(localDisplay[disp],LOW);
      delay(100);
      disp++;
  }
  // initialisation des display externes des pins en output et en écoute (high)
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {/*
        uint16_t msg = j;
        msg|=EXTERNAL_DISPLAY_ON;
        sendI2CMsg(msg, i2c_addr[i]); //mets le premier bite de j à 1 pour mettre j en high
*/
        uint8_t buff[2]={EXTERNAL_DISPLAY_ON,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 

        delay(100);

        display.clearDisplay();
        display.setTextSize(3); // Draw 2X-scale text
       //display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
       // display.println(F("test"));
        display.println(disp);
        display.display();      // Show initial text

        delay(100);
        uint8_t buff2[2]={EXTERNAL_DISPLAY_OFF,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff2,2);
        Wire.endTransmission(); 
        disp++;
      }
  }
}

// TODO test
// void initAxis2() dans l'ordre on s'en fout des joy TODO PAS BON
/*
void initAxis(){
  uint8_t axeCount = 0;
  uint8_t boardCount = 0;

  // Init Axe tab
  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    for (uint8_t j = 0; j<11 && boardCount<BOARD_COUNT;j++)
    {
      axePad[boardCount][axeCount]=i;
      axeNum[boardCount][axeCount]=j;
      axeCount++;
      if (axeCount>=(joystickCount[boardCount]*2+sliderCount[boardCount]+rotaryCount[boardCount]))
      {
        axeCount=0;
        boardCount++;
      }
    }
  }
}

void initAxis2(){

  for (uint8_t i = 0; i<LOCAL_SLIDER_NB; i++)
  {
     pinMode(localSliders[i], INPUT);
  }

    for (uint8_t i = 0; i<(LOCAL_JOY_NB*2); i++)
  {
     pinMode(localJoys[i], INPUT);
  }
  
  uint8_t axeConf[BOARD_COUNT][11];  
  uint8_t padCount = 0;
  uint8_t axeCount = 0;

  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    for (uint8_t j = 0; j<11;j++)
    {
      axeConf[i][j]=1;
    }
  }

  // Fill Axe tab with joy

  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    uint8_t j = 0;
    while(j<(joystickCount[i]) && padCount<BOARD_COUNT && axeCount<11)
    {
      axePad[i][j*2]=padCount;
      axePad[i][(j*2)+1]=padCount;
      axeNum[i][j*2]=axeCount;
      axeConf[padCount][axeCount]=0;
      axeCount++;
      axeNum[i][(j*2)+1]=axeCount;
      axeConf[padCount][axeCount]=0;
      axeCount++;
      if (axeCount==4)
      {
        axeCount=0;
        padCount++;
      }
      j++;
    }
  }


  // Fill Axe tab with slider
  padCount = 0;
  axeCount = 0;
  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    uint8_t j = 0;
    while(j<(sliderCount[i]) && padCount<BOARD_COUNT && axeCount<11)
    {
      while(axeConf[padCount][axeCount]==0)
      {
        axeCount++;
        if(axeCount>10)
        {
          axeCount=0;
          padCount++;
        }
      }
        axePad[i][j]=padCount;
        axeNum[i][j]=axeCount;
        axeConf[padCount][axeCount]=0;
        axeCount++;
        if(axeCount>10){
          axeCount=0;
          padCount++;
        }
        j++;
      }
    }

  // Fill Axe tab with rotary
  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    uint8_t j = 0;
    while(j<(rotaryCount[i]) && padCount<BOARD_COUNT && axeCount<11)
    {
      while(axeConf[padCount][axeCount]==0)
      {
        axeCount++;
        if(axeCount>10)
        {
          axeCount=0;
          padCount++;
        }
      }
      axePad[i][j]=padCount;
      axeNum[i][j]=axeCount;
      axeConf[padCount][axeCount]=0;
      axeCount++;
      if(axeCount>10){
          axeCount=0;
          padCount++;
      }
      j++;
    }
  }
}
*/   

void requestButtonUpdate()
{

  
  requestBoardsUpdate();

  // RAZ rotary encoder TODO create value max rotary et min rotary ?
  /*
  for(int i=rotaryTotal; i<BUTTON_MAX ;i++)
  {
    pads[0].setButton(i,0);

  }
  */
}
/*
void getLocalJoy()
{
  for(uint8_t i=0;i<(LOCAL_JOY_NB*2);i++)
  {
      updateAxis(0,i,analogRead(localJoys[i])/4);
  }
}

void getLocalSlider()
{
  for(uint8_t i=0;i<LOCAL_SLIDER_NB;i++)
  {
      updateAxis(0,LOCAL_JOY_NB*2+i,analogRead(localSliders[i])/4);
  }
}
*/
void getLocalButton(){
  for (uint8_t i =0; i<LOCAL_SINGLE_BUTTON_NB; i++)
  {
    int currentButtonState = !digitalRead(localSingleButton[i]);
    if (currentButtonState != lastLocalButtonState[i])
    {
      //pads[0].setButton(i, currentButtonState); //TODO pas de pads ici < gestion de la config
      if (i==NEXT_DISPLAY && currentButtonState==1)
      {
        if (current_display_conf==DISPLAYCONFCOUNT)
        {
          current_display_conf=0;
        }
        else
        {
          current_display_conf+=1;
        }
        confScreen();
      }
      else if (i==ROT_KNOB && currentButtonState==0)
      {
        razRotKnob();
      }
      else if (i==SWITCH_PULSE && currentButtonState==0)
      {
        razSwitch();
      }
      lastLocalButtonState[i] = currentButtonState;
    }
  }
}

/*
void getLocalRotary(){
  long rotaryRead;
  for(int i = 0; i<rotaryCount[0]; i++){
    rotaryRead = rotaryEncoder[i].read();
    if (rotaryConf[i]==0) //Rotary pulse
    {
      if (rotaryRead<0) {
        pads[0].setButton(rotaryPulse1+(i*2),1);
        rotaryEncoder[i].write(0);
      }
      else if (rotaryRead>0) {
        pads[0].setButton(rotaryPulse1+(i*2)+1,1);
        rotaryEncoder[i].write(0);
      }
    }
    else // Rotary pot
    {
        int rotaryWrite;
        if (rotaryRead>maxRotaryPot)
        {
            rotaryEncoder[i].write(maxRotaryPot);
            rotaryWrite = maxRotaryPot;
        }
        else if(rotaryRead<minRotaryPot)
        {
            rotaryEncoder[i].write(minRotaryPot);
            rotaryWrite = minRotaryPot;
        }
        else
        {
            rotaryWrite=rotaryRead;
        }
        updateAxis(0,i+(joystickCount[0]*2)+sliderCount[0],map(rotaryWrite,minRotaryPot,maxRotaryPot,0,255));
    }
  }
}
*/


void updateAxis(uint8_t board,uint8_t number,uint8_t value) 
{
  switch(number)
  {
    case 0 :  pads[board].setXAxis(value);     break;
    case 1 :  pads[board].setYAxis(value);     break;
    case 2 :  pads[board].setRxAxis(value);    break;
    case 3 :  pads[board].setRyAxis(value);    break;
    case 4 :  pads[board].setZAxis(value);     break;
    case 5 :  pads[board].setRzAxis(value);    break;
    case 6 :  pads[board].setRudder(value);    break;
    case 7 :  pads[board].setThrottle(value);  break;
    case 8 :  pads[board].setAccelerator(value);break;
    case 9 :  pads[board].setBrake(value);     break;
    case 10 : pads[board].setSteering(value);  break;
  }
}

void requestBoardsUpdate(){
  uint8_t data_count;
  uint8_t j;
  uint8_t nbResponse=3;
  uint8_t response[nbResponse];

  for (int i=0;i<BOARD_COUNT;i++)
  {

    j=0;
    Wire.requestFrom(i2c_addr[i], nbResponse);   
    while (j<data_count && Wire.available())
    {
      response[j]=Wire.read();
      j++;
    }

    if (response[0]==TYPE_BUTTON)
    {
      //TODO trouver le numéro global à partir du bouton local
      lastButtonState[i][response[1]]=response[2];
      pads[i].setButton(response[1], response[2]);
    }
    else if (response[0]==TYPE_JOY)
    {
      updateAxis(i, response[1], response[2]);
      lastJoyState[i][response[1]]=response[2];
    }    
    else if (response[0]==TYPE_SLIDER)
    {
      updateAxis(i, joystickCount[i] + response[1], response[2]);
      lastSliderState[i][response[1]]=response[2];
    }    
    else if (response[0]==TYPE_SWITCH2)
    {
      pads[i].setButton(2*response[1]+buttonCount[i]+lastSwitch2State[i][response[1]], 0);
      pads[i].setButton(2*response[1]+buttonCount[i]+response[2], 1);
      lastSwitch2State[i][response[1]]=response[2];
      if(lastLocalButtonState[SWITCH_PULSE])
      {
        buttonRaz=2*response[1]+buttonCount[i]+response[2];
        boardRaz=i;
        doRaz=1;        
      }
    }
    else if (response[0]==TYPE_SWITCH3)
    {
      pads[i].setButton(3*response[1]+buttonCount[i]+switch2Count[i]*2+lastSwitch3State[i][response[1]], 0);
      pads[i].setButton(3*response[1]+buttonCount[i]+switch2Count[i]*2+response[2], 1);
      lastSwitch3State[i][response[1]]=response[2];
      if(lastLocalButtonState[SWITCH_PULSE])
      {
        buttonRaz=3*response[1]+buttonCount[i]+switch2Count[i]*2+response[2];
        boardRaz=i;
        doRaz=1;        
      }
    }    
    else if (response[0]==TYPE_ROTARY)
    {
      if (lastLocalButtonState[ROT_PULSE])
      {
        if ((int8_t) response[2]>0)
        {
          pads[i].setButton(lastRotaryButState[i][response[1]]+buttonCount[i]+switch2Count[i]*2+switch3Count[i]*3+response[1]*2+1, 1);
          buttonRaz=lastRotaryButState[i][response[1]]+buttonCount[i]+switch2Count[i]*2+switch3Count[i]*3+response[1]*2+1;
        }
        else if ((int8_t) response[2]<0)
        {
          pads[i].setButton(lastRotaryButState[i][response[1]]+buttonCount[i]+switch2Count[i]*2+switch3Count[i]*3+response[1]*2, 1);
          buttonRaz=lastRotaryButState[i][response[1]]+buttonCount[i]+switch2Count[i]*2+switch3Count[i]*3+response[1]*2;
        }
        boardRaz=i;
        doRaz=1;
      }
      if (lastLocalButtonState[ROT_AXE])
      {
          if (lastRotaryAxeState[i][response[1]]+(int8_t)response[2]<0)
          {
            lastRotaryAxeState[i][response[1]]=0;
          }
          else if (lastRotaryAxeState[i][response[1]]+(int8_t)response[2]>0)
          {
            lastRotaryAxeState[i][response[1]]=MAX_AXIS;
          }
          else
          {
            lastRotaryAxeState[i][response[1]]=lastRotaryAxeState[i][response[1]]+(int8_t)response[2];
          }
          updateAxis(i,joystickCount[i] + sliderCount[i] + response[1], lastRotaryAxeState[i][response[1]]);
      }
      if (lastLocalButtonState[ROT_KNOB])
      {

        pads[i].setButton(lastRotaryButState[i][response[1]]+buttonCount[i]+switch2Count[i]*2+switch3Count[i]*3+rotaryCount[i]*2+MAX_ROTARY_BUTTON*response[1], 0);
        if ((int8_t) response[2]>0)
        {
          lastRotaryButState[i][response[1]]++;
          if (lastRotaryButState[i][response[1]]==MAX_ROTARY_BUTTON)
          {
            lastRotaryButState[i][response[1]]=0;
          }         
        }
        else if ((int8_t) response[2]<0)
        {
          if (lastRotaryButState[i][response[1]]==0)
          {
            lastRotaryButState[i][response[1]]=MAX_ROTARY_BUTTON-1;
          }  
          else
          {
            lastRotaryButState[i][response[1]]--;
          }       
        }
        pads[i].setButton(lastRotaryButState[i][response[1]]+buttonCount[i]+switch2Count[i]*2+switch3Count[i]*3+rotaryCount[i]*2+MAX_ROTARY_BUTTON*response[1], 1);
      }
    }        
  }
}

/*
void requestMega1Update(){
  uint8_t j=1;
  uint8_t data_count=sliderCount[j]+ joystickCount[j]*2 + rotaryCount[j] +2;

  uint8_t response1[data_count];
  uint8_t i;

  // RECUP DATA MEGA 1
  for(i=0;i<data_count;i++){response1[i]=0;}
  i=0;
  Wire.requestFrom(i2c_addr[j], data_count);   
  while (i<MEGA1_DATA_COUNT && Wire.available()){
    response1[i]=Wire.read();
    i++;
  }
  pads.setRudder(response1[0]);
  pads.setThrottle(response1[1]);
  pads.setAccelerator(response1[2]);
  pads.setBrake(response1[3]);
  i=4;
  while (i<data_count){
    lastButtonState[response1[i]]=response1[i+1];
    pads.setButton(response1[i], response1[i+1]);
    i=i+2;
  }

}

void requestMega2Update(){
  uint8_t j=3;
  uint8_t data_count=sliderCount[j]+ joystickCount[j]*2 + rotaryCount[j] +2;

  uint8_t response1[data_count];
  uint8_t i;

  // RECUP DATA MEGA 1
  for(i=0;i<data_count;i++){response1[i]=0;}
  i=0;
  Wire.requestFrom(i2c_addr[j], data_count);   
  while (i<MEGA1_DATA_COUNT && Wire.available()){
    response1[i]=Wire.read();
    i++;
  }
  i=0;
  while (i<data_count){
    lastButtonState[response1[i]]=response1[i+1];
    pads.setButton(response1[i], response1[i+1]);
    i=i+2;
  }

}
*/
void sendI2CMsg(uint16_t msg, int adresse){ 
  Wire.beginTransmission(adresse); 
  Wire.write((byte *)&msg, 2);          
  Wire.endTransmission(); 
}

void screenDebug(String msg){
      digitalWrite(localDisplay[0],HIGH);
      display.clearDisplay();
      display.setTextSize(5); // Draw 2X-scale text
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      //display.println(F("test"));
      display.println(msg);
      display.display();      // Show initial text
      digitalWrite(localDisplay[0],LOW);
}

void razRotKnob(){
  for (uint8_t i =0;i<BOARD_COUNT;i++)
  {
    for(uint8_t j=0;j<rotaryCount[i];j++)
    {
      lastRotaryButState[i][j]=0;
      for(uint8_t n=0;n<MAX_ROTARY_BUTTON;n++)
      {
        pads[i].setButton(n+buttonCount[i]+switch2Count[i]*2+switch3Count[i]*3+rotaryCount[i]*2+MAX_ROTARY_BUTTON*j, 0);
      }
    }
  }
}

void razSwitch(){
  for (uint8_t i =0;i<BOARD_COUNT;i++)
  {
      for(uint8_t j=0;j<switch2Count[i];j++)
    {
      lastSwitch2State[i][j]=0;
      for(uint8_t n=0;n<3;n++)
      {
        pads[i].setButton(n+buttonCount[i]+2*j, 0);
      }
    }
    for(uint8_t j=0;j<switch3Count[i];j++)
    {
      lastSwitch3State[i][j]=0;
      for(uint8_t n=0;n<3;n++)
      {
        pads[i].setButton(n+buttonCount[i]+switch2Count[i]*2+3*j, 0);
      }
    }
  }
}
