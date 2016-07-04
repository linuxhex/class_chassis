#ifndef COMMON_FUNCTION_H
#define COMMON_FUNCTION_H
#include <string>
#define VERIFY_REMOTE_KEY
#define VERIFY_REMTOE_ID

unsigned int GenerateJSHash(unsigned int);
bool ping(const char*);
void checkConnectionHealthThread(void);
std::string get_key_value(int, int);
void freeResource(void);

#endif // COMMON_FUNCTION_H
