from math import sin, pi

SMPL_NUM = 64

def mapping(num: int) -> int:
    num &= 0x3FFF
    return ((num & 0x2aaa) >> 1) | ((num & 0x1555) << 1)

if __name__ == "__main__":
    raw_fig = [int(round(sin(i * 2 * pi / (SMPL_NUM - 1)) * (2 ** 13))) for i in range(SMPL_NUM)]
    figs = list(map(mapping, raw_fig))
    print(f"{'i':>3}  {'fifo (PIO)':>13}  {'dac (DB13..DB0)':>17}")
    for i, (mp, rw) in enumerate(zip(figs, raw_fig)):
        print(f"{i:3d}  0x{mp:04X} ({mp:5d})  0x{rw:04X} ({rw:+6d})")

    term_str = ''
    for i in figs:
        term_str += (str(f'0x{i:04x}') + ', ')

    print('{' + term_str + '}')