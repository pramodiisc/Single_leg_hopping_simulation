from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.factory import get_problem
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter
from pymoo.problems.functional import FunctionalProblem
from pymoo.factory import get_sampling, get_crossover, get_mutation
from pymoo.factory import get_termination
import csv
from env.single_leg_env import SingleLegEnv
import numpy as np
import matplotlib.pyplot as plt

# problem = get_problem("zdt1")
#Problem: #In pymoo the problem is defined by an object that contains some metadata,
                            # for instance the number of objectives, constraints,
                            # lower and upper bounds in the design space.
env = SingleLegEnv()
# f=open("ga_library_1.txt","w")
env.reset()

def get_reward(action):
    # f=open('policy_log.txt',"a")
    reward_ht=[]
    reward_force=[]
    reward_power=[]
    env.reset()
    for i in range(400):        
        r_ht,r_force,r_power=env.step(action)
        reward_ht.append(r_ht)
        reward_force.append(r_force)
        reward_power.append(r_power)
        # f.write(str(base_position))
        # f.write('\n')
    r1=reward_ht[np.argmax(reward_ht)]
    r2=reward_force[np.argmax(reward_force)]
    r3=sum(reward_power)
    # r3=reward_power[np.argmax(reward_power)]
    # r=[-r1,-r2,-r3]
    r=[-r1,r2,r3]
    print("r1",r1)
    # f.close()
    return r 

r =get_reward
print("get_reward",r)

objs = r
print("obs",objs)


constr_ieq = []
constr_eq = []
n_var = 6

problem = FunctionalProblem(n_var,
                            objs,
                            # constr_ieq=constr_ieq,
                            xl=np.array([150,0,150,0,500,0]),
                            xu=np.array([2000,10,2000,10,100000,20])
                            )

F, *CV = problem.evaluate(np.random.rand(3, 6))
# F is function values
print(f"F: {F}\n")
print(f"CV: {CV}")
termination = get_termination("n_gen", 100)


algorithm = NSGA2(
    pop_size=20,
    n_offsprings=10,
    sampling=get_sampling("real_random"),
    crossover=get_crossover("real_sbx", prob=0.9, eta=15),
    mutation=get_mutation("real_pm", eta=20),
    eliminate_duplicates=True
)

res = minimize(problem,
               algorithm,
               termination,
               seed=1,
               save_history=True,
               verbose=False)


with open("resF.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(res.F)


with open("resX.csv", "w", newline="") as x:
    writer = csv.writer(x)
    writer.writerows(res.X)


plot = Scatter()
plot.add(problem.pareto_front(), plot_type="line", color="black", alpha=0.7)
plot.add(res.F, facecolor="none", edgecolor="red")
print('res.F',res.F)
print('res.X',res.X)
plot.save()
plot.show()


# get the pareto-set and pareto-front for plotting
ps = problem.pareto_set(use_cache=False, flatten=False)
pf = problem.pareto_front(use_cache=False, flatten=False)

# Design Space
plot = Scatter(title = "Design Space", axis_labels="x")
plot.add(res.X, s=30, facecolors='none', edgecolors='r')
if ps is not None:
    plot.add(ps, plot_type="line", color="black", alpha=0.7)
plot.do()
# plot.apply(lambda ax: ax.set_xlim(-0.5, 1.5))
# plot.apply(lambda ax: ax.set_ylim(-2, 2))
plot.save()
plot.show()




# Objective Space
plot = Scatter(title = "Objective Space")
plot.add(res.F)
if pf is not None:
    plot.add(pf, plot_type="line", color="black", alpha=0.7)
plot.save()
plot.show()

#https://pymoo.org/getting_started.html