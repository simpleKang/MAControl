# behavior_v = [c1 c2 c3 c4  w1 w2 w3 w4 w5 w6]

# Rule1：Neighbouring Agent Orientation
# Rule2：Neighbouring Agent Bearing
# Rule3：Neighbouring Target Bearing
# Rule4：Projected Agent Bearing
# Rule5：Projected Target Bearing
# Rule6：Random Orientation

behavior_v = list()

# singleton weights                      w1     w2     w3     w4     w5     w6
behavior_v.append([0.1, 0.1, 0.1, 0.1, 0.500, 0.500, 0.500, 0.500, 0.500, 0.500])
