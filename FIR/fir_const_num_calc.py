import numpy as np
from scipy import signal

num_taps = 64         # タップ数（必ず2の倍数）
fs = 96000             # ターゲットサンプリング周波数
cutoff = 24000         # カットオフ周波数
L = 2                  # アップサンプリング倍率

# 1. 係数の生成（通常の順番）
coeffs = signal.firwin(num_taps, cutoff, fs=fs, window='blackman')

# 2. ゲイン補正（補間によって半減するエネルギーを補うためにL倍する）
coeffs = coeffs * L

# 3. 【超重要】CMSIS-DSP用に「ポリフェーズ並び替え」を行う
# 前半に偶数インデックス、後半に奇数インデックスをまとめる
polyphase_coeffs = np.concatenate((coeffs[0::2], coeffs[1::2]))

# C言語の配列形式で出力
print("const float32_t fir_coeffs[{}] = {{".format(num_taps))
for i in range(0, len(polyphase_coeffs), 4):
    row = polyphase_coeffs[i:i+4]
    print("    " + ", ".join(["{:.8f}f".format(c) for c in row]) + ",")
print("};")