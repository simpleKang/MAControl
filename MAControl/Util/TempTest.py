import argparse
import jsbsim

fdm = jsbsim.FGFDMExec('.', None)
# fdm.load_script('scripts/c1721.xml')
fdm.load_script('E:\\S-Projects\\JSBSim\\jsbsim-Release_Candidate_v1.0.0\\scripts\\c1721.xml')
fdm.run_ic()

while fdm.run():
    pass


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario_paper", help="name of the scenario script")
    parser.add_argument("--step-max", type=int, default=3000, help="maximum steps")
    parser.add_argument("--episode-max", type=int, default=200, help="maximum episodes")
    parser.add_argument("--p1", action='append', type=float, dest='p1', default=[], help="P: Line one")
    parser.add_argument("--p2", action='append', type=float, dest='p2', default=[], help="P: Line Two")
    parser.add_argument("--p3", action='append', type=float, dest='p3', default=[], help="P: Line Three")
    parser.add_argument("--q1", action='append', type=float, dest='q1', default=[], help="Q: Line One")
    parser.add_argument("--q2", action='append', type=float, dest='q2', default=[], help="Q: Line Two")
    parser.add_argument("--q3", action='append', type=float, dest='q3', default=[], help="Q: Line Three")
    parser.add_argument("--numU", type=int, default=20, help="how many UAVs")
    parser.add_argument("--typeT", action='append', type=int, dest='typeT', default=[], help="target types")
    return parser.parse_args()


result = parse_args()
print(result)
print('')
print('scenario:    ', result.scenario)
print('p1:          ', result.p1)
print('p2:          ', result.p2)
print('p3:          ', result.p3)
print('q1:          ', result.q1)
print('q2:          ', result.q2)
print('q3:          ', result.q3)
print('typeT:       ', result.typeT)

