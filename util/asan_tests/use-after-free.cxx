#include "bad_ideas.h"

int main(int, char**) {
	doSomethingWith(misuseHeap(10, 5, true, true));
	return 0;
}
