from env.single_leg_env import SingleLegEnv
import genetic
import time
import numpy as np
import matplotlib.pyplot as plt
from geneticalgorithm import geneticalgorithm as ga

#policy = np.load("experiments/13Oct1 (copy)/iterations/best_policy.npy")
def run():
    env = SingleLegEnv()
    # pos=[]
    # kp=[]
    # pos3=[]
    # actual_position=[]
    f=open("ga_library_1.txt","w")
    env.reset()
    '''
    Every evolutionary algorithm (metaheuristic) has some parameters to be adjusted. 
    Genetic algorithm also has some parameters.
    The parameters of geneticalgorithm is defined as a dictionary:

    '''
    algorithm_param = {'max_num_iteration': 100,\
                    'population_size':150,\
                    'mutation_probability':0.1,\
                    'elit_ratio': 0.01,\
                    'crossover_probability': 0.5,\
                    'parents_portion': 0.3,\
                    'crossover_type':'uniform',\
                    'max_iteration_without_improv':None}

    def get_reward(action):
        reward=[]
        env.reset()
        for i in range(400):        
            r, dev, desired_pos, actual_pos, power, base_position=env.step(action)
            reward.append(r)
        r1=reward[np.argmax(reward)]
        return -r1 #anubhab


    fun=get_reward 

    varbound=np.array([[200,1500],[0,100],[200,1500],[0,100]])
    vartype=np.array([['real'],['real'],['real'],['real']])
    '''
    function fun so that its output is the objective function we want to minimize
    dimension  set of decision variables
    varbound boundaries for variables must be defined as a numpy array and for each variable we need a separate boundary
    '''

    model=ga(function=fun,\
                dimension=4,\
                variable_type_mixed=vartype,\
                variable_boundaries=varbound,\
                algorithm_parameters=algorithm_param)
    model.run()
    '''
    access to the best answer of the defined optimization problem found by geneticalgorithm as a 
    dictionary and a report of the progress of the genetic algorithm
    '''
    convergence=model.report
    solution=model.ouput_dict
    print("solution",solution)
    f.write("Best gain="+str(solution)+"\n")
    f.write(solution)
    f.close()

    '''
    1. frequency also we can learn
    2. Trajectory has to be refined or checked
    3. Foot kp, kd by putting motor wac check
    4. foot and motor damping can be tweeked i.e in motors
    5. Trajectory variation

    '''