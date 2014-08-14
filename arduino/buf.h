#ifndef BUF_H
#define BUF_H

#include <stdlib.h>

template <class T>
struct s_buf {
    volatile unsigned char ptr, size;
    unsigned char max;
    T *v;
};

template <class T>
inline void buf_clear(struct s_buf<T> *b) {
	b->size = 0;
}

template <class T>
void buf_init(struct s_buf<T> *b, unsigned char size) {
        b->ptr = b->size=0;
        b->max = size;
        b->v = (T*)malloc(sizeof(T)*size);
}

// returns space left in the buffer
template <class T>
int buf_space(struct s_buf<T> *b) {
        return (b->max - b->size);
}

template <class T>
void buf_push(struct s_buf<T> *b, T v) {
        if (b->ptr == b->max)
                b->ptr = 0;
        b->v[b->ptr++] = v;
        if (b->size < b->max) b->size++;
}

template <class T>
T buf_pop(struct s_buf<T> *b) {
        static float ret;

        if (!b->size) return 0;

        if (b->ptr<b->size)
                ret = b->v[b->max - b->size + b->ptr];
        else
                ret = b->v[b->ptr - b->size];

        b->size--;

        return ret;
}

#endif

