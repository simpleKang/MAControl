import os

# 算例生成方法 ————
# np.random.choice(3, 10)+1

os.system("python ./execute_Probability.py --numU=20 "
          "--typeT=1 --typeT=3 --typeT=1 --typeT=1 --typeT=3 --typeT=2 --typeT=1 --typeT=2 --typeT=2 --typeT=1 ")

os.system("python ./execute_Probability.py --numU=20 "
          "--typeT=1 --typeT=1 --typeT=2 --typeT=2 --typeT=2 --typeT=3 --typeT=1 --typeT=1 --typeT=1 --typeT=3 ")

os.system("python ./execute_Probability.py --numU=20 "
          "--typeT=2 --typeT=1 --typeT=1 --typeT=2 --typeT=3 --typeT=2 --typeT=3 --typeT=1 --typeT=3 --typeT=1 ")

os.system("python ./execute_Probability.py --numU=20 "
          "--typeT=1 --typeT=2 --typeT=2 --typeT=1 --typeT=3 --typeT=1 --typeT=3 --typeT=3 --typeT=2 --typeT=1 ")

# Will Do # Bernoulli Model & Gilbert-Elliot Model
