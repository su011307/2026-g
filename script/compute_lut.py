from math import sin, pi

SMPL_NUM = 64

def _swap_2bit_pairs(code: int) -> int:
    # PCB 走线决定的 2-bit 组间倒序（7 对）—— GPIO2..15 ↔ DAC_13..DAC_2
    # 是 "组内保序、组间倒序"，与 wavegen.cpp 顶部那段位运算等价。
    # 该操作是自逆的：f(f(x)) == x
    r = 0
    for i in range(7):
        r |= ((code >> (12 - 2 * i)) & 0x3) << (2 * i)
    return r


def mapping(num: int) -> int:
    # wavegen.cpp:37 明确"最高位为符号位"——AD9744 处于 14-bit two's complement 模式。
    # 直接取 signed 值的低 14 位 bit pattern，正数即本身，负数自动出两's complement 编码。
    # 范围 [-8192, +8191]；bit 13 = 1 视为负。
    code = num & 0x3FFF
    return _swap_2bit_pairs(code)


figs = [mapping(int(round(sin(i * 2 * pi / (SMPL_NUM - 1)) * (2 ** 13))))
        for i in range(SMPL_NUM)]

if __name__ == "__main__":
    # 预览：FIFO 列是贴进 C 数组的值；DAC 列是 AD9744 实际收到的 code（两's complement 下
    # 0x1FFF ~ +8191 满正，0x2000 ~ -8192 满负，0 正好是中点）
    print(f"{'i':>3}  {'fifo (PIO)':>13}  {'dac (DB13..DB0)':>17}")
    for i, v in enumerate(figs):
        dac = _swap_2bit_pairs(v)
        # 还原成有符号十进制
        dac_signed = dac - 0x4000 if dac & 0x2000 else dac
        print(f"{i:3d}  0x{v:04X} ({v:5d})  0x{dac:04X} ({dac_signed:+6d})")
