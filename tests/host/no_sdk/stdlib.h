#ifndef HOST_NO_SDK_STDLIB_H
#define HOST_NO_SDK_STDLIB_H
#define NULL ((void *)0)
static __inline int abs(int value) { return value < 0 ? -value : value; }
long strtol(const char *, char **, int);
#endif
