#include "bad_ideas.h"
char globalString[] = "Woe is me! Hi, Woe, I'm Dad!";
int main(int, char**) {
	unsigned int index = 0;
	// build up index by loop so the overflow can't be detected at compile time
	for (int ii = 0; ii < 10; ++ii) index += 10;
	doSomethingWith(globalString[index]);
	return 0;
}
