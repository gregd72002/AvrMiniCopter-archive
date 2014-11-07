#ifndef BUF_H
#define BUF_H

#include <stdlib.h>

template <class T>
struct s_buf {
	volatile unsigned int ptr, size;
	unsigned int max;
	T *v;
};

template <class T>
inline void buf_clear(struct s_buf<T> *b) {
	b->size = 0;
}

template <class T>
void buf_init(struct s_buf<T> *b, unsigned int size) {
	b->ptr = b->size=0;
	b->max = size;
	b->v = (T*)malloc(sizeof(T)*size);
}

template <class T>
void buf_free(struct s_buf<T> *b) {
	free(b->v);
}

// returns space left in the buffer
template <class T>
unsigned int buf_space(struct s_buf<T> *b) {
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
void buf_push(struct s_buf<T> *b, T *v, unsigned int c) {
	for (unsigned int i=0;i<c;i++) {
		if (b->ptr == b->max)
			b->ptr = 0;
		b->v[b->ptr++] = v[i];
		if (b->size < b->max) b->size++;
	}
}

template <class T>
T buf_pop(struct s_buf<T> *b) {
	static T ret;

	if (!b->size) return 0;

	if (b->ptr<b->size)
		ret = b->v[b->max - b->size + b->ptr];
	else
		ret = b->v[b->ptr - b->size];

	b->size--;

	return ret;
}

#endif

