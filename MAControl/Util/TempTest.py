import argparse


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario_paper", help="name of the scenario script")
    parser.add_argument("--step-max", type=int, default=3000, help="maximum steps")
    parser.add_argument("--episode-max", type=int, default=200, help="maximum episodes")
    parser.add_argument("-p1", action='append', dest='p1', default=[], help="P: Line one")
    parser.add_argument("-p2", action='append', dest='p2', default=[], help="P: Line Two")
    parser.add_argument("-p3", action='append', dest='p3', default=[], help="P: Line Three")
    parser.add_argument("-q1", action='append', dest='q1', default=[], help="Q: Line One")
    parser.add_argument("-q2", action='append', dest='q2', default=[], help="Q: Line Two")
    parser.add_argument("-q3", action='append', dest='q3', default=[], help="Q: Line Three")
    parser.add_argument("--numU", type=int, default=20, help="how many UAVs")
    parser.add_argument("-typeT", action='append', dest='typeT', default=[], help="target types")
    return parser.parse_args()


result = parse_args()
print(result)
print('')
print('scenario:    ', result.scenario)

