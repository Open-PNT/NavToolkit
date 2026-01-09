from navtk.filtering import StateBlock, StandardDynamicsModel

from aspn23_xtensor import to_seconds


class BiasBlock(StateBlock):
    def __init__(self, label):
        StateBlock.__init__(self, num_states=1, label=label)

    def clone(self):
        return BiasBlock(self.label)

    # This function takes in the current state estimate and time
    # and produces the needed dynamics equation parameters (g, Phi, Qd)
    def generate_dynamics(self, xhat, time_from, time_to):

        # Define the g(x) propagation function as g(x_(k)) = x_(k-1)
        def g(x):
            return x

        delta_time = to_seconds(time_to - time_from)

        Phi = [[1]]
        Qd = [[delta_time]]

        return StandardDynamicsModel(g, Phi, Qd)
