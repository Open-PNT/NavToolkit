from numpy import zeros

from navtk.inertial import calc_force_ned, StandardPosVelAtt
from navtk.utils import to_navsolution, linear_interpolate, to_inertial_aux
from navtk.filtering import apply_error_states


def time_min(t1, t2):
    if t1 < t2:
        return t1
    else:
        return t2


def create_pinson15_ins_aux_data(ins, imu, imu_dt):
    ins_pva = ins.get_solution()
    f_ned = calc_force_ned(
        ins_pva.get_C_s_to_ned(),
        imu_dt,
        imu.get_meas_gyro(),
        imu.get_meas_accel(),
    )
    return to_inertial_aux(to_navsolution(ins_pva), f_ned, zeros((3)))


def create_pva_estimate(ins, engine, use_this_lla, pinson_block_name):
    ins_pva = ins.get_solution()
    mod_pva = StandardPosVelAtt(
        engine.get_time(),
        use_this_lla,
        ins_pva.get_vned(),
        ins_pva.get_C_s_to_ned(),
    )
    x_pinson = engine.get_state_block_estimate(pinson_block_name)
    return apply_error_states(mod_pva, x_pinson)


def interpolate_inertial_lla(next_imu, tmp_ins, time):
    last_pva = tmp_ins.get_solution()
    last_lla = last_pva.get_llh()
    tmp_ins.mechanize(
        next_imu.get_time_of_validity(),
        next_imu.get_meas_accel(),
        next_imu.get_meas_gyro(),
    )
    next_pva = tmp_ins.get_solution()
    next_lla = next_pva.get_llh()
    return linear_interpolate(
        last_pva.time_validity,
        last_lla,
        next_pva.time_validity,
        next_lla,
        time,
    )
