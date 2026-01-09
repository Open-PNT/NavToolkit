#include <memory>
#include <string>

#include <navtoolkit.hpp>

// Define a new VirtualStateBlock
class MetersToFeetVirtualStateBlock : public navtk::filtering::VirtualStateBlock {
public:
	MetersToFeetVirtualStateBlock(std::string current, std::string target)
	    : VirtualStateBlock(std::move(current), std::move(target)) {}

	navtk::Matrix jacobian(const navtk::Vector&, const aspn_xtensor::TypeTimestamp&) override {
		return navtk::Matrix{{3.28084}};
	}

	navtk::Vector convert_estimate(const navtk::Vector& x,
	                               const aspn_xtensor::TypeTimestamp&) override {
		return 3.28084 * x;
	}

	navtk::not_null<std::shared_ptr<VirtualStateBlock>> clone() override {
		return std::make_shared<MetersToFeetVirtualStateBlock>(*this);
	}
};

int main() {
	// Then create elsewhere
	auto vsb = MetersToFeetVirtualStateBlock("speed_meters", "speed_feet");
}
