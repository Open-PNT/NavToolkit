// ASAN_OPTIONS=detect_leaks=1
int main(int, char**) {
	auto ptr = new char[10];
	ptr      = 0;
	return 0;
}
