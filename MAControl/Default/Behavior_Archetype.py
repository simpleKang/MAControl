# BA_b = [b.C1  b.C2  b.W1  b.W2  b.W3  b.W4  b.W5  b.W6  b.W7  b.W8]
# seen_uav # rule1 rule3 rule4 rule9 # 列队 靠近 远离 预判远离
# seen_target # rule2 rule6 rule8 # 切向 靠近 远离

BA = list()

#                       w1    w2    w3    w4    w5    w6    w7    w8
BA.append([0.99, 0.99, 0.77, 0.01, 0.77, 0.77, 0.99, 0.01, 0.77, 0.01])
BA.append([0.99, 0.99, 0.01, 0.77, 0.01, 0.01, 0.99, 0.77, 0.01, 0.77])
