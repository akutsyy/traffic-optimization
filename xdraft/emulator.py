import pickle

import GPy
import numpy as np
from emukit.core.loop import UserFunctionWrapper
from emukit.model_wrappers.gpy_model_wrappers import GPyModelWrapper
from emukit.experimental_design.acquisitions import ModelVariance
from emukit.core.optimization import GradientAcquisitionOptimizer
from emukit.examples.gp_bayesian_optimization.single_objective_bayesian_optimization import GPBayesianOptimization
from emukit.bayesian_optimization.acquisitions import ExpectedImprovement
from emukit.core import ParameterSpace, ContinuousParameter, DiscreteParameter
import runner
import matplotlib.pyplot as plt

FOURLIGHTS = [ContinuousParameter('traffic_light_1', 1, 100),
              ContinuousParameter('traffic_light_2', 1, 100),
              ContinuousParameter('traffic_light_3', 1, 100),
              ContinuousParameter('traffic_light_4', 1, 100)]

INITIAL_PARTIAL_VARIABLES = [0.5, 0.5, 1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]


class emu():
    def __init__(self, from_pickle=None):


        # Define parameter space for the simulator variables
        self.space = ParameterSpace([ContinuousParameter('traffic_light_1', 1, 10),
                                     ContinuousParameter('traffic_light_2', 1, 10),
                                     ContinuousParameter('traffic_light_3', 1, 10),
                                     ContinuousParameter('traffic_light_4', 1, 10),
                                     ContinuousParameter('sigma', 0.5, 0.5),
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
        kern = GPy.kern.RBF(21, lengthscale=0.08, variance=20)

        # GP
        self.X = np.array([[1, 1, 1, 1] + INITIAL_PARTIAL_VARIABLES])
        self.Y = np.array(runner.call_sim_parallel(self.X))
        gpy_model = GPy.models.GPRegression(self.X, self.Y, kern, noise_var=1e-10)
        # Emukit Model
        if from_pickle:
            with open(from_pickle, 'rb') as f:
                self.model = pickle.load(f)
                print("Successfullly unpickled")
        else:
            self.model = GPyModelWrapper(gpy_model)

    # Save to file if given a name
    def explore(self, iterations,batch_size, save_filename=None):
        # Acquisition function
        us_acquisition = ModelVariance(self.model)
        # Acquisition optimiser
        optimizer = GradientAcquisitionOptimizer(self.space)

        for i in range(iterations):
            # Get next point
            global x_new
            x_new, _ = optimizer.optimize(us_acquisition)

            # Run simulator on new
            y_new = np.array([runner.call_sim(list(x_new[0]))])

            self.X = np.append(self.X, x_new, axis=0)
            self.Y = np.append(self.Y, np.expand_dims(y_new, 0), axis=0)

            print(f"X: {x_new}")
            print(f"Y: {y_new}")

            # Add data to model
            self.model.set_data(np.array(self.X), np.array(self.Y))

            if i % 10 == 9 and save_filename:
                with open(save_filename, 'wb') as pw:
                    pickle.dump(self.model, pw)
                    print("Dumped model succesfully at iteration " + str(i))

        if save_filename:
            with open(save_filename, 'wb') as pw:
                pickle.dump(self.model, pw)
                print("Dumped model succesfully at last iteration")

    def call(self, x):
        # Returns tuple of mean and variance
        return self.model.predict(x)

    # partial_x will be things like number of cars on the road
    def optimise(self, partial_x, to_optimize=FOURLIGHTS, iterations=50, maximize=True):
        assert len(partial_x) == 17
        if maximize:
            sign = -1
        else:
            sign = 1

        # optimise_x are the parameters to optimise
        def partial_call(optimise_x):
            return sign * self.call(np.expand_dims(np.append(optimise_x, partial_x), axis=0))[0]

        user_function = UserFunctionWrapper(partial_call)
        x_init = np.array([np.ones(len(to_optimize))])
        y_init = partial_call(x_init)

        bo = GPBayesianOptimization(variables_list=to_optimize,
                                    X=x_init, Y=y_init)
        bo.run_optimization(user_function, iterations)

        print(bo.loop_state.X)
        print(bo.loop_state.Y)
        print(bo.loop_state.results)
        maximum = bo.loop_state.X[-1]
        return maximum, bo.loop_state

if __name__ == '__main__':
    picklename = "emulator_test_many_iterations.pkl"
    explore_iterations = 10000
    optimize_iterations = 50
    from_pickle = False

    if from_pickle:
        e = emu(picklename)
    else:
        e = emu()
        e.explore(explore_iterations, save_filename=picklename)

    max_values, loop_state = e.optimise(INITIAL_PARTIAL_VARIABLES, iterations=optimize_iterations)
    xs = range(optimize_iterations + 1)
    ys = [-1 * y[0] for y in loop_state.Y]

    fig, ax = plt.subplots()
    ax.plot(xs, ys)

    ax.set(xlabel='Iteration of Bayesian Optimization', ylabel='Mean Speed (% of speed limit)',
           title='Mean Car Speed After ' + str(optimize_iterations) + ' Iterations of Bayesian Optimization')
    ax.grid()

    fig.savefig("bo_results.png")
    plt.show()

    fig, ax = plt.subplots()

    ys = loop_state.X
    newys = []
    for y in ys:
        sum = np.sum(y)
        newys.append(100 * y / sum)
    ys = np.array(newys).transpose()
    print(ys)

    for y in ys:
        ax.plot(xs, y)

    ax.set(xlabel='Iteration of Bayesian Optimization', ylabel='Percent of time spent in each light configuration',
           title='Traffic Light Configuration After ' + str(
               optimize_iterations) + ' Iterations of Bayesian Optimization')
    ax.grid()
    fig.savefig("bo_config.png")
    plt.show()
