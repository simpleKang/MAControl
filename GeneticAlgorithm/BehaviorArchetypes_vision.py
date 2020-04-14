# behavior_v = [C1  C2  W1  W3  W4]

# W1: Alignment
# W2: Target Orbit
# W3: Cohesion
# W4: Separation
# W5: Take turns when losing sight of target
# W6: Flat Target Repulsion
# W7: (deleted)
# W8: Flat Target Attraction
# W9: Evasion

behavior_v = list()
# behavior_v = [     C1     C2     W1     W3     W4]

# rule 1+3+4+9
behavior_v.append([0.880, 0.880, 1.000, 1.000, 0.500])


