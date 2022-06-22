import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

PI = np.pi
PI2 = PI * PI

t_arr = np.linspace(0, 2 * PI, 10001)

p_arr = np.linspace(0, PI * np.sqrt(2), 1001)

test_arr = np.linspace(0, PI2, 10001)


def pos(t=t_arr):
    return t * t / 2 + np.cos(t) - 1


def sqpos(t=t_arr):
    return np.sqrt(pos(t))


def test(num, kind="linear"):
    tt_arr = np.linspace(0, 2 * PI, num)
    p_int = interp1d(pos(tt_arr), tt_arr, kind=kind)
    sq_int = interp1d(sqpos(tt_arr), tt_arr, kind=kind)

    pp = pos(t_arr)
    plt.plot(t_arr, pp, "--")
    # plt.plot(p_int(pp), pp)
    plt.plot(sq_int(np.sqrt(pp)), pp)


def sq_table(num, kind="linear"):
    tt_arr = np.linspace(0, 2 * PI, 10001)
    sq_int = interp1d(sqpos(tt_arr), tt_arr, kind=kind)
    return sq_int(np.linspace(0, PI * np.sqrt(2), num))


def table(num, kind="linear"):
    tt_arr = np.linspace(0, 2 * PI, 10001)
    p_int = interp1d(pos(tt_arr), tt_arr, kind=kind)
    return p_int(np.linspace(0, PI2 * 2, num))

