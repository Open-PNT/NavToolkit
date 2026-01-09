// looking_for=heap-buffer
#include "bad_ideas.h"

int main(int, char**) {
	doSomethingWith(misuseHeap(10, -5));
	return 0;
}
