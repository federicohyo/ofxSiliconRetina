#ifndef THREADS_EXT_H_
#define THREADS_EXT_H_

#ifdef __cplusplus

#include <cstdlib>
#include <cstdint>

#else

#include <stdlib.h>
#include <stdint.h>

#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <sys/time.h>

#if defined(__linux__)
	#include <sys/prctl.h>
	#include <sys/resource.h>
#endif

enum {
	thrd_success = 0, thrd_error = 1, thrd_nomem = 2, thrd_timedout = 3, thrd_busy = 4,
};

// NON STANDARD!
static inline int thrd_set_name(const char *name) {
#if defined(__linux__)
	if (prctl(PR_SET_NAME, name) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#elif defined(__APPLE__)
	if (pthread_setname_np(name) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#else
	(void)(name); // UNUSED.

	return (thrd_error);
#endif
}

static inline int thrd_get_name(char *name, size_t maxNameLength) {
#if defined(__linux__)
	(void)(maxNameLength); // UNUSED ON LINUX.

	if (prctl(PR_GET_NAME, name) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#elif defined(__APPLE__)
	if (pthread_getname_np(pthread_self(), name, maxNameLength) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#else
	(void)(name); // UNUSED.
	(void)(maxNameLength); // UNUSED.

	return (thrd_error);
#endif
}

// NON STANDARD!
static inline int thrd_set_priority(int priority) {
#if defined(__linux__)
	if (setpriority(PRIO_PROCESS, 0, priority) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#else
	(void)(priority); // UNUSED.

	return (thrd_error);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* THREADS_EXT_H_ */
