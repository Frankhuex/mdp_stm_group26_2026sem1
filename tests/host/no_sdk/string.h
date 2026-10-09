#ifndef HOST_STRING_H
#define HOST_STRING_H
#include <stddef.h>
size_t strlen(const char *);
void *memset(void *, int, size_t);
void *memcpy(void *, const void *, size_t);
int strcmp(const char *, const char *);
int strncmp(const char *, const char *, size_t);
char *strchr(const char *, int);
char *strstr(const char *, const char *);
char *strtok(char *, const char *);
#endif
