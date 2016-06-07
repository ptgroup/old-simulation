#include "helper.h"

#include <string.h>

void strip_newline(char *str) {
    int i = 0;
    while (str[i] != '\n' && str[i] != '\r') i++;
    str[i] = '\0';
}

void strip_extension(char *str) {
    int i = strlen(str) - 1;
    while (str[i] != '.') i--;
    str[i] = '\0';
}
