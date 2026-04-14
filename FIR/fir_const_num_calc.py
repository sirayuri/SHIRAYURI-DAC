import numpy as np
from scipy import signal

num_taps = 64
fs = 96000
L = 2

# Parks-McClellan設計
coeffs = signal.remez(num_taps, [0, 20000, 28000, 48000], [1, 0], fs=fs) * L

# 配列全体をただ逆順にするだけ
polyphase_coeffs = coeffs[::-1]

print(f"const float32_t fir_coeffs[{num_taps}] = {{")
for i in range(0, len(polyphase_coeffs), 4):
    row = polyphase_coeffs[i:i+4]
    print("    " + ", ".join(["{:.8f}f".format(c) for c in row]) + ",")
print("};")