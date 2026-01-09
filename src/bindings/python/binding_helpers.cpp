#include "binding_helpers.hpp"

using std::regex;
using std::string;

string docfmt(const string& docstring) {
	auto out = docstring;
	out      = std::regex_replace(out, regex("([^\\s\n])\n([^\n])"), "$01 $02");
	out      = std::regex_replace(out, regex("([^\n])\n"), "$01");
	out      = std::regex_replace(out, regex("@"), "\n@");
	return out;
}
