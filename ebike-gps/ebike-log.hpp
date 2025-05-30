#pragma once

#include "ebike-conf.h"
#include <Arduino.h>
#include "modem_utilities.h"

#define EBIKE_LOG_HD Serial.printf("[%lu] ", millis() / 1000)

template <typename T>
void ebike_va_print(T t)
{
  Serial.print(t);
}

template <typename T, typename... Args>
void ebike_va_print(T t, Args... args)
{
  Serial.print(t);
  ebike_va_print(args...);
}

#define EBIKE_LOG(type, msg, ...)     \
  EBIKE_LOG_HD;                       \
  Serial.print("[" type "] ");        \
  ebike_va_print(msg, ##__VA_ARGS__); \
  Serial.println("")

#define EBIKE_LOGF(type, msg, ...)   \
  EBIKE_LOG_HD;                      \
  Serial.print("[" type "] ");       \
  Serial.printf(msg, ##__VA_ARGS__); \
  Serial.println("")

#ifdef EBIKE_DEBUG_BUILD
#define EBIKE_ERR(msg, ...) EBIKE_LOG("ERR", msg, ##__VA_ARGS__)
#define EBIKE_NFO(msg, ...) EBIKE_LOG("NFO", msg, ##__VA_ARGS__)
#define EBIKE_ERRF(msg, ...) EBIKE_LOGF("ERR", msg, ##__VA_ARGS__)
#define EBIKE_NFOF(msg, ...) EBIKE_LOGF("NFO", msg, ##__VA_ARGS__)
#define EBIKE_DBG(msg, ...) EBIKE_LOG("DBG", msg, ##__VA_ARGS__)
#define EBIKE_DBGF(msg, ...) EBIKE_LOGF("DBG", msg, ##__VA_ARGS__)
#else
#define EBIKE_ERR(msg, ...)
#define EBIKE_NFO(msg, ...)
#define EBIKE_ERRF(msg, ...)
#define EBIKE_NFOF(msg, ...)
#define EBIKE_DBG(msg, ...)
#define EBIKE_DBGF(msg, ...)
#endif // EBIKE_DEBUG