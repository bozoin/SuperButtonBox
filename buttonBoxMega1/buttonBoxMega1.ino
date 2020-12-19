#include <LinkedList.h>
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
Envoyer les inputs rotary telquel > traiter par leonardo
ANNULER utilisation de liste. Trop lourd, création en mémoire trop souvent.

Un seul emplacement ? si déja utilisé on ne rajoute rien > au prochain tour il comparera toujours positif avec précédent état et rajoutera si emplacement libéré
*/

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
#define LOCAL_SINGLE_BUTTON_NB 1
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
#define SLIDER_COUNT 4
#define FIRST_BUTTON 40

// TEST
#define pinTest 9
#define sliderTest A2

// MASQUE
const uint8_t MASQUE_5b=0x1F;
#define DISPLAY_ON 0x88
#define DISPLAY_OFF 0x80

const uint8_t totalButton = LOCAL_SINGLE_BUTTON_NB + LOCAL_ROTARY_COUNT*10 + localTotalMatriceButton;

// Last state of the button
uint8_t lastButtonState[totalButton];
uint8_t lastSliderState[SLIDER_COUNT];

class buttonDeclare
{
  public :
  int button;
  uint8_t state;
  uint8_t decompte;
};

/*
struct pButtonDeclare
{
  buttonDeclare * buttonToDeclare;
}pButtonDeclare; 
*/
LinkedList<buttonDeclare> *listToDeclare = new LinkedList<buttonDeclare>();

void setup()
{
  Serial.begin(9600); //DEBUG

  Wire.begin(I2C_SLAVE_ADDRESS);
  delay(1000);               
  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);

  for (int disp = 0; disp<LOCAL_DISPLAY_NB;disp++ )
  {
    pinMode(localDisplay[disp],OUTPUT);
    digitalWrite(localDisplay[disp],LOW);
  }

    // Init Button Pins
  pinMode(pinTest, INPUT_PULLUP);
  pinMode(sliderTest, INPUT);
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
  int offsetButton= LOCAL_SINGLE_BUTTON_NB;

  for (int i=0;i<LOCAL_MATRICE_COUNT;i++)
  {
    char key = matrices[i].getKey();   //Surveiller les boutons
    if (key != NO_KEY){

      buttonDeclare nouveauBouton;
      nouveauBouton.button=key+offsetButton;
      nouveauBouton.state=1;
      nouveauBouton.decompte=0;
      listToDeclare->add(nouveauBouton);
    }
    offsetButton=offsetButton+localCountMatriceButton[i];
  }
  delay(T_CYCLE);  
}

void requestEvents()
{
  uint8_t slider[SLIDER_COUNT];
  for(int i = 0; i<SLIDER_COUNT;i++){slider[i]=analogRead(sliderTest)/4;}
  for(int i = 0; i<SLIDER_COUNT;i++){Wire.write(slider[i]);}
  Wire.write(40);
  Wire.write(1);

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




/*


//==== Utilisation de clavier matriciel 4 x4 boutons
// tiptopboards.com
// Rolland 23 08 2013
// télécharger et installer la librairie Keypad
//
//===================================================
#include <Keypad.h>

//Définition du clavier 4 x4 boutons
const byte ROWS = 4; //4 lignes
const byte COLS = 4; //4 colonnes
// Donner un code Ascii aux 16 touches, par exemple 0-F
char keys[ROWS][COLS] = {
  {'1','2','3','4'},
  {'5','6','7','8'},
  {'9','10','A','B'},
  {'C','D','E','F'}
};
byte rowPins[ROWS] = {2, 3, 4, 5}; //les 4 lignes du clavier sur les pins arduino digitales
byte colPins[COLS] = {6, 7, 8, 9}; //les 4 colonnes du clavier

//Créer cet objet clavier
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

void setup(){
  Serial.begin(9600);   //Moniteur série (écran PC)
  Serial.println("Utilisation du clavier 4 x f4 touches");
}

void loop(){
  char key = keypad.getKey();   //Surveiller le calvier

  if (key != NO_KEY){
    Serial.println(key);  //Afficher le code ascii de la touche appuyée
  }
}


*/



/* LINKED LIST

To declare a LinkedList object

// Instantiate a LinkedList that will hold 'integer'
LinkedList<int> myLinkedList = LinkedList<int>();

// Or just this
LinkedList<int> myLinkedList;

// But if you are instantiating a pointer LinkedList...
LinkedList<int> *myLinkedList = new LinkedList<int>();

// If you want a LinkedList with any other type such as 'MyClass'
// Make sure you call delete(MyClass) when you remove!
LinkedList<MyClass> *myLinkedList = new LinkedList<MyClass>();

Getting the size of the linked list

// To get the size of a linked list, make use of the size() method
int theSize = myList.size();

// Notice that if it's pointer to the linked list, you should use -> instead
int theSize = myList->size();

Adding elements

// add(obj) method will insert at the END of the list
myList.add(myObject);

// add(index, obj) method will try to insert the object at the specified index
myList.add(0, myObject); // Add at the beginning
myList.add(3, myObject); // Add at index 3

// unshift(obj) method will insert the object at the beginning
myList.unshift(myObject);

Getting elements

// get(index) will return the element at index
// (notice that the start element is 0, not 1)

// Get the FIRST element
myObject = myList.get(0);

// Get the third element
myObject = myList.get(2);

// Get the LAST element
myObject = myList.get(myList.size() - 1);

Changing elements

// set(index, obj) method will change the object at index to obj

// Change the first element to myObject
myList.set(0, myObject);

// Change the third element to myObject
myList.set(2, myObject);

// Change the LAST element of the list
myList.set(myList.size() - 1, myObject);

Removing elements

// remove(index) will remove and return the element at index

// Remove the first object
myList.remove(0);

// Get and Delete the third element
myDeletedObject = myList.remove(2);

// pop() will remove and return the LAST element
myDeletedObject = myList.pop();

// shift() will remove and return the FIRST element
myDeletedObject = myList.shift();

// clear() will erase the entire list, leaving it with 0 elements
// NOTE: Clear wont DELETE/FREE memory from Pointers, if you
// are using Classes/Poiners, manualy delete and free those.
myList.clear();

Sorting elements

// Sort using a comparator function
myList.sort(myComparator);
*/
