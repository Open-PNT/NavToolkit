from navtk.filtering import MeasurementProcessor, StandardMeasurementModel


class BiasMeasurementProcessor(MeasurementProcessor):
    def __init__(self, label, state_block_label):
        MeasurementProcessor.__init__(self, label, state_block_label)
        self.measurement_matrix = [[1.0]]

    def generate_model(self, meas, gen_X_and_P_func):
        z = meas.estimate
        H = self.measurement_matrix
        R = meas.covariance
        return StandardMeasurementModel(z, H, R)

    def clone(self):
        BiasMeasurementProcessor(self.label, self.state_block_labels[0])
