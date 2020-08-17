const int sensorPin = 0; //The analog sensor is connected to analog pin 0 of the arduino

//ABCDEFG,dp
const int numeral[10] = {
  B11111100, //0
  B01100000, //1
  B11011010, //2
  B11110010, //3
  B01100110, //4
  B10110110, //5
  B10111110, //6
  B11100000, //7
  B11111110, //8
  B11110110, //9
};

#define ON_TIME 2  // number of millis the segments will light up

//pins for decimal point and each segment
//dp, G, F, E, D, C, B, A
const int segmentPins[] = { 1, 8, 7, 6, 5, 4, 3, 2};

const int numberofDigits = 4;

const int digitPins[numberofDigits] = { 12, 11, 10, 9}; //digits 1, 2, 3, 4

void setup()
{
  Serial.begin(9600);

  for (int i = 0; i < 8; i++)
    pinMode(segmentPins[i], OUTPUT); //set segment and DP pins to output

  //sets the digit pins as outputs
  for (int i = 0; i < numberofDigits; i++)
    pinMode(digitPins[i], OUTPUT);
}

void loop()
{
  showNumber(millis() / 10);
}

void showNumber (int number)
{
  if (number == 0)
  {
    showDigit(0, numberofDigits - 1); //display 0 in the rightmost digit
  } else
  {
    for (int digit = numberofDigits - 1; digit >= 0; digit--)
    {
      if (number > 0)
      {
        showDigit(number % 10, digit);
        number = number / 10;
      }
    }
  }
}

//Displays given number on a 7-segment display at the given digit position
void showDigit (int number, int digit)
{
  for (int segment = 1; segment < 8; segment++)
  {
    boolean isBitSet = bitRead(numeral[number], segment);

    //    isBitSet = ! isBitSet; //remove this line if common cathode display
    digitalWrite(segmentPins[segment], isBitSet);
  }
  digitalWrite(digitPins[digit], HIGH);
  delay(ON_TIME);
  digitalWrite(digitPins[digit], LOW);
}
