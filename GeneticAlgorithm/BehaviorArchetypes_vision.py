# behavior_v = [c1 c2 c3 c4  w1 w2 w3 w4 w5 w6]

# Rule1：Neighbouring Agent Orientation (~ alignment)
# Rule2：Neighbouring Agent Bearing
# Rule3：Neighbouring Target Bearing
# Rule4：Projected Agent Bearing (~ projection)
# Rule5：Projected Target Bearing
# Rule6：Random Orientation (~ noise)

behavior_v = list()

# singleton weights                      w1     w2     w3     w4     w5     w6
behavior_v.append([0.1, 0.1, 0.1, 0.1, 0.750, 0.000, 0.000, 0.100, 0.000, 0.150])  # RoleProjControlBirdFlock P3 Fig2 A
