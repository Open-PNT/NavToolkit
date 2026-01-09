#pragma once
#include <cassert>

char misuseHeap(char bytes, int offset, bool initialize = true, bool freeBeforeRead = false) {
	auto heap = new char[bytes];
	for (char ii = 0; initialize && ii < bytes; ++ii) heap[ii] = ii + 'a';
	if (freeBeforeRead) delete[] heap;
	return heap[offset];
}

template <char bytes>
char misuseStack(int offset, bool initialize = true) {
	char stack[bytes];
	for (char ii = 0; initialize && ii < bytes; ++ii) stack[ii] = ii + 'a';
	return stack[offset];
}

__attribute__((noinline)) void setPointerToStack(char*& ptr) {
	char stack[26];
	for (char ii = 0; ii < 26; ++ii) stack[ii] = 'a' + ii;
	ptr = stack;
}

template <typename T>
void doSomethingWith(T&& t) {
	auto a = t;
	assert(a == t);
}
