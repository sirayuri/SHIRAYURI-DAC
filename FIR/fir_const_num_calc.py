import numpy as np
from scipy import signal

num_taps = 64          # タップ数
fs = 96000             # 出力FS
L = 2                  # 倍率

# 1. Parks-McClellan (Remez) アルゴリズムで「攻めた」設計をする
# [0, 20kHz]はフラットに、[28kHz, 48kHz]は完全に遮断
bands = [0, 20000, 28000, 48000]
desired = [1, 0]
coeffs = signal.remez(num_taps, bands, desired, fs=fs)

# 2. 補間によるレベル低下を補正 (L倍)
coeffs = coeffs * L

# 3. 【最重要】CMSIS-DSP専用の「インターリーブ・リバース」並び替え
# 各フェーズ（偶数・奇数）を取り出して、それぞれを逆順にする
phase0_rev = coeffs[0::2][::-1]
phase1_rev = coeffs[1::2][::-1]

# 交互に配置して一つの配列にする
polyphase_coeffs = np.empty(num_taps, dtype=np.float32)
polyphase_coeffs[0::2] = phase0_rev
polyphase_coeffs[1::2] = phase1_rev

# C言語形式で出力
print(f"const float32_t fir_coeffs[{num_taps}] = {{")
for i in range(0, len(polyphase_coeffs), 4):
    row = polyphase_coeffs[i:i+4]
    print("    " + ", ".join(["{:.8f}f".format(c) for c in row]) + ",")
print("};")