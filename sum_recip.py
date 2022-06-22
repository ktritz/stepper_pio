# approximation of sum(1/i)  1 < i < n
from math import log, exp

gamma = 0.577215664901532860606512


def rsum(n):
    return log(n) + 1 / n + gamma * (1 + log(n / (n + 1)))


def asum(n):
    return log(n) + gamma


def isum(t, d):
    dd = t / d
    pl = dd * exp(gamma)
    return dd / (log(pl) - log(log(pl)) + log(log(pl)) / log(pl))

