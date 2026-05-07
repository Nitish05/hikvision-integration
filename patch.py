from painting_bridge.safety import wrap_deg
target = [0, 0, 0, 170, 0, 0]
anchor = [0, 0, 0, 170, 0, 0]
target[3] = 190
out = anchor[3] + wrap_deg(target[3] - anchor[3])
print(f"Output of normalize: {out}")
