#ifndef READERS_WRITERS_LOCK
#define READERS_WRITERS_LOCK

#include <pthread.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>

typedef struct {
        pthread_mutex_t *mut;
        int writers;
        int readers;
        int waiting;
        pthread_cond_t *writeOK, *readOK;
} rwl;

rwl * initLock (void);
void readLock (rwl *lock);
void writeLock (rwl *lock);
void readUnlock (rwl *lock);
void writeUnlock (rwl *lock);
void deleteLock (rwl *lock);

#endif
