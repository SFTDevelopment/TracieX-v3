/* Input Command class:
  Purpose: To construct command arguments from string input
  Input is sent letter by letter. This class keeps every
  letter until a newline symbol is found and then tokenizes
  the string to find the method name and arguments.
  Input: None
  Private variables:
  - buffer_counts: How many letters have been read so far in this command
  - separator: separator of method's and argument's arguments
  - buffer_string: where the chars are stored.
  - message_validity: If the message is smaller than the max length (for now).
  - token: pointer to access new arguments during tokeization.
  - received_char: char to get the serial input letter.
  Public fields:
  - method_name = name of the method.
  - argument_1/2/3/4: up to four arguments. char arrays.
  - message_validity: true if the command string complies with the format.
  Methods:
  - receive_character: if available, read the next character incoming from
   the serial port and saves it in buffer_string. When it detects an end of
   line, it tokenizes the string looking for method name and max four arguments
  - tokenize: separate the buffer_string into mathod_name and arguments
  - restart: resets the command string variables to initial values
  - print_character: prints some text for debugging.
*/

#ifndef InputCommand_h
#define InputCommand_h
#include "Arduino.h"
#define MAX_STRING_SIZE 32

class InputCommand
{
  private:
    /* PRIVATE FIEDLS */
    int buffer_counts = 0;
    const char separator[2] = " ";
    char buffer_string[MAX_STRING_SIZE] = "";
    char *token;
    char received_char;

  public:
    /* PUBLIC FIELDS */
    void _restart();
    bool tokenize();
    bool receive_character();
    void print_character(char character, int count);
    bool message_validity = true;
    char method_name[MAX_STRING_SIZE+1] = "";
    char argument_1[MAX_STRING_SIZE+1] = "";
    char argument_2[MAX_STRING_SIZE+1] = "";
    char argument_3[MAX_STRING_SIZE+1] = "";
    char argument_4[MAX_STRING_SIZE+1] = "";
};

void InputCommand::_restart() {
  /*method to reset values to be ready for next incoming string*/
  buffer_counts = 0;
  message_validity = true;
  for (int i = 0; i < MAX_STRING_SIZE; i++) {
    buffer_string[i] = '\0';
  }
}

bool InputCommand::receive_character() {
  /*method to receive a new character from serial input*/

  if (Serial.available() > 0) {
    // read the incoming byte:
    received_char = Serial.read();

    // say what you got:
    //print_character(received_char, buffer_counts);
    // abort if command message is too long.
    if (buffer_counts == MAX_STRING_SIZE) {
      message_validity = false;
      Serial.println("input command too long. Aborted.");
    }
    // wrap up if end of line character
    if (received_char == '\n') {
      if (message_validity) {
//        Serial.println("end of message");
//        Serial.println(buffer_string);
        bool result_tokenize = tokenize();
        if (result_tokenize) {
          return (true);
        }
        else {
          _restart();
          return (false);
        }
      }
      else {
        _restart();
        return (false);
      }
    }
    else {
      // append character to buffer string
      buffer_string[buffer_counts] = received_char;
      buffer_counts += 1;
    }
  }
  return (false);
}

//bool InputCommand::tokenize() {
//  /*tokenize input string into maethod name and argument names*/
//
//  // get method name
//  token = strtok(buffer_string, separator);
//  for (int i = 0; i < MAX_STRING_SIZE; i++) {
//    method_name[i] = token[i];
//  }
//  //get first argument
//  token = strtok(NULL, separator);
//  for (int i = 0; i < MAX_STRING_SIZE; i++) {
//    argument_1[i] = token[i];
//  }
//  //get second argument
//  token = strtok(NULL, separator);
//  for (int i = 0; i < MAX_STRING_SIZE; i++) {
//    argument_2[i] = token[i];
//  }
//  //get third argument
//  token = strtok(NULL, separator);
//  for (int i = 0; i < MAX_STRING_SIZE; i++) {
//    argument_3[i] = token[i];
//  }
//  //get fourth argument
//  token = strtok(NULL, separator);
//  for (int i = 0; i < MAX_STRING_SIZE; i++) {
//    argument_4[i] = token[i];
//  }
//  // print aguments
//#ifdef DEBUGGING_MODE
//  Serial.print("The method is: ");
//  Serial.println(method_name);
//  Serial.println("The arguments are: ");
//  Serial.print(argument_1);
//  Serial.print(", ");
//  Serial.print(argument_2);
//  Serial.print(", ");
//  Serial.print(argument_3);
//  Serial.print(", ");
//  Serial.println(argument_4);
//#endif
//  return (true);
//}

bool InputCommand::tokenize()
{
  char *token;

  // get method name
  token = strtok(buffer_string, separator);

  if (NULL == token)
  {
    memset(method_name, '\0', sizeof(method_name));
    return false;
  }
  strncpy(method_name, token, sizeof (method_name));

  //since arguments are optional, don't return false on empty token

  //get first argument
  token = strtok(NULL, separator);
  if (NULL != token)
  {
    strncpy(argument_1, token, sizeof (argument_1));
  }
  else
  {
    memset(argument_1, '\0', sizeof(argument_1));
  }

  //get second argument
  token = strtok(NULL, separator);
  if (NULL != token)
  {
    strncpy(argument_2, token, sizeof (argument_2));
  }
  else
  {
    memset(argument_2, '\0', sizeof(argument_2));
  }
  //get third argument
  token = strtok(NULL, separator);
  if (NULL != token)
  {
    strncpy(argument_3, token, sizeof (argument_3));
  }
  else
  {
    memset(argument_3, '\0', sizeof(argument_3));
  }
  //get fourth argument
  token = strtok(NULL, separator);
  if (NULL != token)
  {
    strncpy(argument_4, token, sizeof (argument_4));
  }
  else
  {
    memset(argument_4, '\0', sizeof(argument_4));
  }
  // print aguments
#ifdef DEBUGGING_MODE
  Serial.print("Method : ");
  Serial.println(method);
  Serial.print("Arguments : ");
  Serial.print(argument_1);
  Serial.print(",");
  Serial.print(argument_2);
  Serial.print(",");
  Serial.print(argument_3);
  Serial.print(",");
  Serial.println(argument_4);
#endif
  return (true);
}

void InputCommand::print_character(char character, int count) {
  /* print every accepted character */
  Serial.print("I received: ");
  Serial.print(character);
  Serial.print(" ");
  Serial.println(count);
}

#endif
