#include <navtk/filtering/containers/StandardDynamicsModel.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;

namespace navtk {
namespace filtering {

StandardDynamicsModel::StandardDynamicsModel(StateTransitionFunction g, Matrix Phi, Matrix Qd)
    : g(std::move(g)), Phi(std::move(Phi)), Qd(std::move(Qd)) {
	ValidationContext{}
	    .add_matrix(this->Phi, "Phi")
	    .dim('M', 'M')
	    .add_matrix(this->Qd, "Qd")
	    .dim('M', 'M')
	    .validate();
}

StandardDynamicsModel::StandardDynamicsModel(Matrix Phi, Matrix Qd)
    : g([=](const Vector& x) { return dot(Phi, x); }), Phi(std::move(Phi)), Qd(std::move(Qd)) {
	ValidationContext{}
	    .add_matrix(this->Phi, "Phi")
	    .dim('M', 'M')
	    .add_matrix(this->Qd, "Qd")
	    .dim('M', 'M')
	    .validate();
}

}  // namespace filtering
}  // namespace navtk
