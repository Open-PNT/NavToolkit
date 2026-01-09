#include "bad_ideas.h"

int main(int, char**) {
	doSomethingWith(misuseStack<10>(15));
	return 0;
}
