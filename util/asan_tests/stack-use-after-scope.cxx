#include "bad_ideas.h"

int main(int, char**) {
	char* ptr;
	{
		char foo = 'a';
		ptr      = &foo;
	}
	doSomethingWith(*ptr = 'b');
	return 0;
}
