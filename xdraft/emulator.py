import pickle

import GPy
import numpy as np
from emukit.core.initial_designs import RandomDesign
from emukit.core.loop import UserFunctionWrapper
from emukit.model_wrappers.gpy_model_wrappers import GPyModelWrapper
from emukit.experimental_design.acquisitions import ModelVariance
from emukit.core.optimization import GradientAcquisitionOptimizer
from emukit.examples.gp_bayesian_optimization.single_objective_bayesian_optimization import GPBayesianOptimization
from emukit.bayesian_optimization.acquisitions import ExpectedImprovement
from emukit.core import ParameterSpace, ContinuousParameter, DiscreteParameter
from emukit.experimental_design.experimental_design_loop import ExperimentalDesignLoop
from emukit.bayesian_optimization.loops import BayesianOptimizationLoop

import runner
import matplotlib.pyplot as plt

FOURLIGHTS = [ContinuousParameter('traffic_light_1', .1, .5),
              ContinuousParameter('traffic_light_2', .1, .5),
              ContinuousParameter('traffic_light_3', .1, .5),
              ContinuousParameter('traffic_light_4', .1, .5)]

INITIAL_PARTIAL_VARIABLES = [0.5, 0.5, 1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]


class emu():
    def __init__(self, from_pickle=None):

        # Define parameter space for the simulator variables

        self.space = ParameterSpace(FOURLIGHTS + [ContinuousParameter('sigma', 0.5, 0.5),
                                                  ContinuousParameter('tau', 0.5, 0.5),
                                                  ContinuousParameter('trucks', 1, 1),
                                                  ContinuousParameter('cars', 1, 1),
                                                  ContinuousParameter('bikes', 1, 1),
                                                  ContinuousParameter('NE', 0.01, 0.5),
                                                  ContinuousParameter('NS', 0.01, 0.5),
                                                  ContinuousParameter('NW', 0.01, 0.5),
                                                  ContinuousParameter('EN', 0.01, 0.5),
                                                  ContinuousParameter('ES', 0.01, 0.5),
                                                  ContinuousParameter('EW', 0.01, 0.5),
                                                  ContinuousParameter('SN', 0.01, 0.5),
                                                  ContinuousParameter('SE', 0.01, 0.5),
                                                  ContinuousParameter('SW', 0.01, 0.5),
                                                  ContinuousParameter('WN', 0.01, 0.5),
                                                  ContinuousParameter('WE', 0.01, 0.5),
                                                  ContinuousParameter('WS', 0.01, 0.5),
                                                  ])

        # Kernel
        kern = GPy.kern.RBF(21, lengthscale=0.25, variance=0.001)

        # GP
        self.X = np.array([[.1, .1, .1, .1] + INITIAL_PARTIAL_VARIABLES])
        self.Y = np.array(runner.call_sim_parallel(self.X))
        gpy_model = GPy.models.GPRegression(self.X, self.Y, kern, noise_var=1e-10)
        # TODO: ?????????????
        gpy_model.optimize()
        # Emukit Model
        if from_pickle:
            with open(from_pickle, 'rb') as f:
                self.model = pickle.load(f)
                print("Successfullly unpickled")
        else:
            self.model = GPyModelWrapper(gpy_model)

    # Save to file if given a name
    def explore(self, iterations, batch_size, save_filename=None):
        # Acquisition function
        us_acquisition = ModelVariance(self.model)
        # Acquisition optimiser
        optimizer = GradientAcquisitionOptimizer(self.space)

        expdesign_loop = ExperimentalDesignLoop(space=self.space,
                                                model=self.model,
                                                acquisition=us_acquisition,
                                                update_interval=1,
                                                acquisition_optimizer=optimizer,
                                                batch_size=batch_size)

        # Run the loop
        for i in range(iterations // 50):
            expdesign_loop.run_loop(runner.call_sim_parallel, 50 // batch_size)
            with open(save_filename, 'wb') as pw:
                pickle.dump(self.model, pw)
                print("Dumped model succesfully at iteration " + str((i + 1) * 50))

        if save_filename:
            with open(save_filename, 'wb') as pw:
                pickle.dump(self.model, pw)
                print("Dumped model succesfully at last iteration")

    def call(self, x):
        # Returns tuple of mean and variance
        return self.model.predict(x)

    def call_split(self, x, rest):
        return self.call(np.expand_dims(np.append(x, rest), axis=0))

    # partial_x will be things like number of cars on the road
    def optimise(self, partial_x, to_optimize=FOURLIGHTS, iterations=50):
        assert len(partial_x) == 17


        # optimise_x are the parameters to optimise
        def partial_call(optimise_x):
            output = -self.call(np.expand_dims(np.append(optimise_x, partial_x), axis=0))[0]
            return output

        user_function = UserFunctionWrapper(partial_call)

        design = RandomDesign(ParameterSpace(to_optimize))
        x_init = design.get_samples(1)
        y_init = partial_call(x_init)

        bo = GPBayesianOptimization(variables_list=to_optimize,
                                    X=x_init, Y=y_init,batch_size=1)
        bo.run_optimization(user_function, iterations)

        best_per_it = np.maximum.accumulate(-1*bo.loop_state.Y)
        best_result = np.max(-1*bo.loop_state.Y)
        best_config = bo.loop_state.X[np.argmax(-1*bo.loop_state.Y)]
        return best_result,best_config,best_per_it,bo.loop_state.X,bo.loop_state.Y*-1


if __name__ == '__main__':
    picklename = "emulator_test_many_iterations.pkl"
    explore_iterations = 400
    optimize_iterations = 50
    from_pickle = True

    if from_pickle:
        e = emu(picklename)
    else:
        e = emu()
        e.explore(explore_iterations, save_filename=picklename, batch_size=5)

    #num_tests = 50
    #xs = np.linspace(.1,.5,num_tests)
    #ys = []
    #for x in xs:
    #    ys.append(e.call_split([0.2,0.2,x,0.2],[0.5, 0.5, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])[0])

    #fig, ax = plt.subplots()
    #ax.plot(xs, ys)
    #plt.show()

    best_result,best_config,best_per_it, loop_x,loop_y = e.optimise(INITIAL_PARTIAL_VARIABLES, iterations=optimize_iterations)

    xs = range(optimize_iterations+1)
    ys = 100*best_per_it
    print(best_config)
    print(runner.call_sim(params=np.append(best_config,INITIAL_PARTIAL_VARIABLES),name="test"))

    fig, ax = plt.subplots()
    ax.plot(xs, ys)

    ax.set(xlabel='Iteration of Bayesian Optimization', ylabel='Highest Discovered Mean Speed (% of speed limit)',
           title='Discovery of Best Configuration with Bayesian Optimization')
    ax.grid()

    fig.savefig("bo_results.png")
    plt.show()

    fig, ax = plt.subplots()

    ys = loop_x
    newys = []
    for y in ys:
        sum = np.sum(y)
        newys.append(100 * y / sum)
    ys = np.array(newys).transpose()

    for y in ys:
        ax.plot(xs, y)

    ax.set(xlabel='Iteration of Bayesian Optimization', ylabel='Percent of time spent in each light configuration',
           title='Exploration of Parameter Space during Bayesian Optimization')
    ax.grid()
    fig.savefig("bo_config.png")
    plt.show()
