// script.h --- for reading from a simple script file
#ifndef _SCRIPT_H
#define _SCRIPT_H

#include <stdbool.h>

int script_fopen(char *filename); // returns 0 on success
int script_readline(); // returns number of characters (in command or argument)
bool script_cmdequ(char *command); // whether the command of the line is *command*
char *script_getarg(int n); // returns an argument
void script_fclose(); // closes the file

#endif
