#include "helper.h"

#include <stdlib.h>
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

int get_port(char *port_name) {
    int len = strlen(port_name);
    if (len >= 4 && port_name[0] == 'C' && port_name[1] == 'O' && port_name[2] == 'M') {
        return atoi(port_name + 3);
    } else {
        return -1;
    }
}
