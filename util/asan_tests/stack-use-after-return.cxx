// ASAN_OPTIONS=detect_stack_use_after_return=1

#include "bad_ideas.h"

int main(int, char**) {
	char* ptr;
	setPointerToStack(ptr);
	doSomethingWith(ptr[5]);
	return 0;
}
