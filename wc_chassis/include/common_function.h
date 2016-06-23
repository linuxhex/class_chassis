#ifndef COMMON_FUNCTION_H
#define COMMON_FUNCTION_H

//#define VERIFY_REMOTE_KEY
#define VERIFY_REMTOE_ID

unsigned int GenerateJSHash(unsigned int seed);
bool ping(const char* ip);
void checkConnectionHealthThread(void);

#endif // COMMON_FUNCTION_H
