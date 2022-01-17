import GPy
import numpy as np
from emukit.core.loop import UserFunctionWrapper
from emukit.model_wrappers.gpy_model_wrappers import GPyModelWrapper
from emukit.experimental_design.acquisitions import ModelVariance
from emukit.core.optimization import GradientAcquisitionOptimizer
from emukit.examples.gp_bayesian_optimization.single_objective_bayesian_optimization import GPBayesianOptimization
from emukit.core import ParameterSpace, ContinuousParameter, DiscreteParameter
from runner import call_sim

FOURLIGHTS = [ContinuousParameter('traffic_light_1', 1, 100),
              ContinuousParameter('traffic_light_2', 1, 100),
              ContinuousParameter('traffic_light_3', 1, 100),
              ContinuousParameter('traffic_light_4', 1, 100)]


class emu():
    def __init__(self, iterations):
        # Define parameter space for the simulator variables
        self.space = ParameterSpace([ContinuousParameter('traffic_light_1', 1, 100),
                                     ContinuousParameter('traffic_light_2', 1, 100),
                                     ContinuousParameter('traffic_light_3', 1, 100),
                                     ContinuousParameter('traffic_light_4', 1, 100),
                                     ContinuousParameter('sigma', 0, 1),
                                     ContinuousParameter('tau', 0.01, 1000),  # Don't know how to do no upper bound
                                     ContinuousParameter('trucks', 0, 100),
                                     ContinuousParameter('cars', 0, 100),
                                     ContinuousParameter('bikes', 0, 100),
                                     ContinuousParameter('NE', 0, 0.5),
                                     ContinuousParameter('NS', 0, 0.5),
                                     ContinuousParameter('NW', 0, 0.5),
                                     ContinuousParameter('EN', 0, 0.5),
                                     ContinuousParameter('ES', 0, 0.5),
                                     ContinuousParameter('EW', 0, 0.5),
                                     ContinuousParameter('SN', 0, 0.5),
                                     ContinuousParameter('SE', 0, 0.5),
                                     ContinuousParameter('SW', 0, 0.5),
                                     ContinuousParameter('WN', 0, 0.5),
                                     ContinuousParameter('WE', 0, 0.5),
                                     ContinuousParameter('WS', 0, 0.5),
                                     ])

        # Kernel
        self.kern = GPy.kern.RBF(21, lengthscale=0.08, variance=20)

        # GP
        self.X = np.array([[1, 1, 1, 1, 0, 0.01, 2, 2, 2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]])
        self.Y = np.array([[call_sim(self.X[0])]])
        self.gpy_model = GPy.models.GPRegression(self.X, self.Y, self.kern, noise_var=1e-10)
        # Emukit Model
        self.model = GPyModelWrapper(self.gpy_model)

    def train(self, iterations):
        # Acquisition function
        us_acquisition = ModelVariance(self.model)
        # Acquisition optimiser
        optimizer = GradientAcquisitionOptimizer(self.space)

        for _ in range(iterations):
            # Get next point
            x_new, _ = optimizer.optimize(us_acquisition)

            # Run simulator on new
            y_new = np.array([call_sim(list(x_new[0]), early_stop=True)])

            self.X = np.append(self.X, x_new, axis=0)
            self.Y = np.append(self.Y, np.expand_dims(y_new, 0), axis=0)

            print(f"X: {x_new}")
            print(f"Y: {y_new}")

            # Add data to model
            self.model.set_data(np.array(self.X), np.array(self.Y))

    def call(self, x):
        # Returns tuple of mean and variance
        return self.model.predict(x)

    # partial_x will be things like number of cars on the road
    def optimise(self, partial_x, to_optimize=FOURLIGHTS, iterations=10):
        # optimise_x are the parameters to optimise
        def partial_call(optimise_x):
            return self.call(partial_x + optimise_x)[0]

        user_function = UserFunctionWrapper(partial_call)

        bo = GPBayesianOptimization(variables_list=to_optimize,
                                    X=X_init, Y=Y_init)
        bo.run_optimization(user_function, iterations)

        maximum = bo.loop_state.X[np.argmax(bo.loop_state.Y)]
        return maximum


emu(10)
