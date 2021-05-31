# behavior_v = [c1 c2 c3 c4  w1 w2 w3 w4 w5 w6]

# Rule1：Neighbouring Agent Orientation (~ alignment)
# Rule2：Neighbouring Agent Bearing
# Rule3：Neighbouring Target Bearing
# Rule4：Projected Agent Bearing (~ projection)
# Rule5：Projected Target Bearing
# Rule6：Random Orientation (~ noise)

behavior_v = list()

# singleton weights                      w1     w2     w3     w4     w5     w6
behavior_v.append([0.1, 0.1, 0.1, 0.1, 0.750, 0.000, 0.000, 0.100, 0.000, 0.150])  # RoleProjControlBirdFlock Fig2 (A)
behavior_v.append([0.1, 0.1, 0.1, 0.1, 0.700, 0.000, -1.00, 0.200, 0.000, 0.100])  # RPCBF Supp.Mov.S9
behavior_v.append([0.1, 0.1, 0.1, 0.1, 0.700, 0.000, +1.00, 0.200, 0.000, 0.100])  # RPCBF Supp.Mov.S9 Reverse
