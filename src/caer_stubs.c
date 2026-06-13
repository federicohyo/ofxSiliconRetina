/* No-op stubs for libcaer logging functions — not linking against libcaer .so */
#include <stdarg.h>
#include <stdint.h>

enum caer_log_level { CAER_LOG_EMERGENCY=0, CAER_LOG_ALERT, CAER_LOG_CRITICAL,
                      CAER_LOG_ERROR, CAER_LOG_WARNING, CAER_LOG_NOTICE,
                      CAER_LOG_INFO, CAER_LOG_DEBUG };

void caerLog(enum caer_log_level logLevel, const char *subSystem, const char *format, ...) {
    (void)logLevel; (void)subSystem; (void)format;
}

void caerLogVA(enum caer_log_level logLevel, const char *subSystem, const char *format, va_list ap) {
    (void)logLevel; (void)subSystem; (void)format; (void)ap;
}

void caerLogVAFull(uint8_t sysLevel, enum caer_log_level logLevel,
                   const char *subSystem, const char *format, va_list ap) {
    (void)sysLevel; (void)logLevel; (void)subSystem; (void)format; (void)ap;
}
