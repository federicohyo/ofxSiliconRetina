#ifndef LIBCAER_HPP_
#define LIBCAER_HPP_

#include <libcaer/libcaer.h>
#include <stdexcept>
#include <type_traits>

namespace libcaer {
namespace log {

// Undefine the log-level names, to avoid conflicts with macros
// on Windows/MinGW for example.
#undef EMERGENCY
#undef ALERT
#undef CRITICAL
#undef ERROR
#undef WARNING
#undef NOTICE
#undef INFO
#undef DEBUG

enum class logLevel {
	EMERGENCY = 0,
	ALERT = 1,
	CRITICAL = 2,
	ERROR = 3,
	WARNING = 4,
	NOTICE = 5,
	INFO = 6,
	DEBUG = 7,
};

inline void logLevelSet(logLevel l) noexcept;
inline logLevel logLevelGet() noexcept;
inline void fileDescriptorsSet(int fd1, int fd2) noexcept;
inline int fileDescriptorsGetFirst() noexcept;
inline int fileDescriptorsGetSecond() noexcept;
inline void log(logLevel l, const char *subSystem, const char *format, ...) noexcept;
inline void logVA(logLevel l, const char *subSystem, const char *format, va_list args) noexcept;
inline void logVAFull(int logFileDescriptor1, int logFileDescriptor2, uint8_t systemLogLevel, logLevel l,
	const char *subSystem, const char *format, va_list args) noexcept;

inline void logLevelSet(logLevel l) noexcept {
	caerLogLevelSet(static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)));
}

inline logLevel logLevelGet() noexcept {
	return (static_cast<logLevel>(caerLogLevelGet()));
}

inline void fileDescriptorsSet(int fd1, int fd2) noexcept {
	caerLogFileDescriptorsSet(fd1, fd2);
}

inline int fileDescriptorsGetFirst() noexcept {
	return (caerLogFileDescriptorsGetFirst());
}

inline int fileDescriptorsGetSecond() noexcept {
	return (caerLogFileDescriptorsGetSecond());
}

inline void log(logLevel l, const char *subSystem, const char *format, ...) noexcept {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVA(static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)),
		subSystem, format, argumentList);
	va_end(argumentList);
}

inline void logVA(logLevel l, const char *subSystem, const char *format, va_list args) noexcept {
	caerLogVA(static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)),
		subSystem, format, args);
}

inline void logVAFull(int logFileDescriptor1, int logFileDescriptor2, uint8_t systemLogLevel, logLevel l,
	const char *subSystem, const char *format, va_list args) noexcept {
	caerLogVAFull(logFileDescriptor1, logFileDescriptor2, systemLogLevel,
		static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)), subSystem,
		format, args);
}

}
}

#endif /* LIBCAER_HPP_ */
