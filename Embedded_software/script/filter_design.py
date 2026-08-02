from scipy.signal import firwin

# --- 用户需定义的参数 ---
SAMPLE_RATE = 65_000_000
CUTOFF_FREQ = 550_000
NUM_TAPS = 65 # 阶数+1。数值越大，过渡带越陡峭，但计算量也越大

# --- 设计FIR低通滤波器 ---
# 归一化截止频率：截止频率 / (采样率/2)
nyquist_freq = 0.5 * SAMPLE_RATE
normalized_cutoff = CUTOFF_FREQ / nyquist_freq

# 使用firwin设计滤波器，采用汉明窗 (hamming window)
# 这与Mathematica中LowpassFilter的默认窗函数一致[reference:2]
coefficients = firwin(NUM_TAPS, normalized_cutoff, window='hamming')

def main() -> None:
    # --- 将系数格式化为C数组，方便复制到C++代码中 ---
    # 打印为C语言的浮点数组格式
    c_array_string = "{" + ", ".join([f"{coef:.8e}f" for coef in coefficients]) + "}"
    print(f"#define NUM_TAPS {NUM_TAPS}")
    print(f"static float32_t firCoeffs[NUM_TAPS] = {c_array_string};")


if __name__ == "__main__":
    main()
