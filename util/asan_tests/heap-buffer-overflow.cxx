#include "bad_ideas.h"

int main(int, char**) {
	doSomethingWith(misuseHeap(10, 15));
	return 0;
}
