/**
 * @file log.h
 *
 * Logging functions to print useful messages for the user.
 */

#ifndef LIBCAER_LOG_H_
#define LIBCAER_LOG_H_

#ifdef __cplusplus

#	include <cstdarg>
#	include <cstdint>
#	include <cstddef>

#else

#	include <stdarg.h>
#	include <stdint.h>
#	include <stddef.h>

#endif

// printf() like formatting checks.
#if defined(__GNUC__) || defined(__clang__)
#	if defined(__USE_MINGW_ANSI_STDIO)
#		define ATTRIBUTE_FORMAT(N)    __attribute__((format(gnu_printf, N, (N + 1))))
#		define ATTRIBUTE_FORMAT_VA(N) __attribute__((format(gnu_printf, N, 0)))
#	else
#		define ATTRIBUTE_FORMAT(N)    __attribute__((format(printf, N, (N + 1))))
#		define ATTRIBUTE_FORMAT_VA(N) __attribute__((format(printf, N, 0)))
#	endif
#else
#	define ATTRIBUTE_FORMAT(N)
#	define ATTRIBUTE_FORMAT_VA(N)
#endif

// Shared objects visibility (DLL/SO).
#if defined(_WIN32) || defined(_WIN64) || defined(__CYGWIN__)
#	ifdef __GNUC__
#		define LIBRARY_PUBLIC_VISIBILITY __attribute__((dllexport)) __attribute__((cdecl))
#	else
#		define LIBRARY_PUBLIC_VISIBILITY __declspec(dllexport)
#	endif
#else
#	define LIBRARY_PUBLIC_VISIBILITY __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Log levels for caerLog() logging function.
 * Log messages only get printed if their log level is equal or
 * above the global system log level, which can be set with
 * caerLogLevelSet().
 * The default log level is CAER_LOG_ERROR.
 * CAER_LOG_EMERGENCY is the most urgent log level and will always
 * be printed, while CAER_LOG_DEBUG is the least urgent log
 * level and will only be delivered if configured by the user.
 */
enum caer_log_level {
	CAER_LOG_EMERGENCY = 0,
	CAER_LOG_ALERT     = 1,
	CAER_LOG_CRITICAL  = 2,
	CAER_LOG_ERROR     = 3,
	CAER_LOG_WARNING   = 4,
	CAER_LOG_NOTICE    = 5,
	CAER_LOG_INFO      = 6,
	CAER_LOG_DEBUG     = 7,
};

/**
 * Set the system-wide log level.
 * Log messages will only be printed if their level is equal or above
 * this level.
 *
 * @param logLevel the system-wide log level.
 */
LIBRARY_PUBLIC_VISIBILITY void caerLogLevelSet(enum caer_log_level logLevel);

/**
 * Get the current system-wide log level.
 * Log messages are only printed if their level is equal or above
 * this level.
 *
 * @return the current system-wide log level.
 */
LIBRARY_PUBLIC_VISIBILITY enum caer_log_level caerLogLevelGet(void);

/**
 * Logging callback, called on any caerLog() invocation.
 * Arguments are the full log string resulting from the
 * caerLog() calls, plus its size (excluding trailing NUL byte).
 */
typedef void (*caerLogCallback)(const char *msg, size_t msgLength);

/**
 * Set callback function to be used on each log message.
 *
 * @param callback the callback function. NULL to disable.
 */
LIBRARY_PUBLIC_VISIBILITY void caerLogCallbackSet(caerLogCallback callback);

/**
 * Get current callback function for log messages.
 */
LIBRARY_PUBLIC_VISIBILITY caerLogCallback caerLogCallbackGet(void);

/**
 * Set to which file descriptors log messages are sent.
 * Up to two different file descriptors can be configured here.
 * By default logging to STDERR only is enabled.
 * If both file descriptors are identical, logging to it will
 * only happen once, as if the second one was disabled.
 *
 * @param fd1 first file descriptor to log to. A negative value will disable it.
 * @param fd2 second file descriptor to log to. A negative value will disable it.
 */
LIBRARY_PUBLIC_VISIBILITY void caerLogFileDescriptorsSet(int fd1, int fd2);

/**
 * Get the current output file descriptor 1.
 *
 * @return the current output file descriptor 1.
 */
LIBRARY_PUBLIC_VISIBILITY int caerLogFileDescriptorsGetFirst(void);

/**
 * Get the current output file descriptor 2.
 *
 * @return the current output file descriptor 2.
 */
LIBRARY_PUBLIC_VISIBILITY int caerLogFileDescriptorsGetSecond(void);

/**
 * Disable all logging for this thread only.
 * Call again with different argument to re-enable.
 *
 * @param disableLogging true to disable logging for this thread,
 *                       false to enable it again.
 */
LIBRARY_PUBLIC_VISIBILITY void caerLogDisable(bool disableLogging);

/**
 * Status of logging for this thread.
 *
 * @return true if logging is disabled for this thread,
 *         false if it is enabled.
 */
LIBRARY_PUBLIC_VISIBILITY bool caerLogDisabled(void);

/**
 * Main logging function.
 * This function takes messages, formats them and sends them out to a file descriptor,
 * respecting the system-wide log level setting and prepending the current time, the
 * log level and a user-specified common string to the actual formatted output.
 * The format is specified exactly as with the printf() family of functions.
 * Please see their manual-page for more information.
 *
 * @param logLevel the message-specific log level.
 * @param subSystem a common, user-specified string to prepend before the message.
 * @param format the message format string (see printf()).
 * @param ... the parameters to be formatted according to the format string (see printf()).
 */
LIBRARY_PUBLIC_VISIBILITY void caerLog(enum caer_log_level logLevel, const char *subSystem, const char *format, ...)
	ATTRIBUTE_FORMAT(3);

/**
 * Secondary logging function.
 * This function takes messages, formats them and sends them out to a file descriptor,
 * respecting the system-wide log level setting and prepending the current time, the
 * log level and a user-specified common string to the actual formatted output.
 * The format is specified exactly as with the printf() family of functions.
 * The argument list is a va_list as returned by va_start(), following the vprintf()
 * family of functions in its functionality.
 * Please see their manual-page for more information.
 *
 * @param logLevel the message-specific log level.
 * @param subSystem a common, user-specified string to prepend before the message.
 * @param format the message format string (see printf()).
 * @param args the parameters to be formatted according to the format string (see printf()).
 *             This is an argument list as returned by va_start().
 */
LIBRARY_PUBLIC_VISIBILITY void caerLogVA(
	enum caer_log_level logLevel, const char *subSystem, const char *format, va_list args) ATTRIBUTE_FORMAT_VA(3);

/**
 * Tertiary logging function.
 * This function takes messages, formats them and sends them out via up to
 * two file descriptors and a callback; allows a user-given system log level
 * setting to also be specified, and then prepends the current time, the message
 * log level and a user-specified common string to the actual formatted output.
 * The format is specified exactly as with the printf() family of functions.
 * The argument list is a va_list as returned by va_start(), following the vprintf()
 * family of functions in its functionality.
 * Please see their manual-page for more information.
 *
 * @param systemLogLevel the system-wide log level.
 * @param logLevel the message-specific log level.
 * @param subSystem a common, user-specified string to prepend before the message.
 * @param format the message format string (see printf()).
 * @param args the parameters to be formatted according to the format string (see printf()).
 *             This is an argument list as returned by va_start().
 */
LIBRARY_PUBLIC_VISIBILITY void caerLogVAFull(uint8_t systemLogLevel, enum caer_log_level logLevel,
	const char *subSystem, const char *format, va_list args) ATTRIBUTE_FORMAT_VA(4);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_LOG_H_ */
