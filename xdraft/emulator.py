import GPy
import numpy as np
from emukit.model_wrappers.gpy_model_wrappers import GPyModelWrapper
from emukit.experimental_design.acquisitions import ModelVariance
from emukit.core.optimization import GradientAcquisitionOptimizer
from emukit.examples.gp_bayesian_optimization.single_objective_bayesian_optimization import GPBayesianOptimization
from emukit.bayesian_optimization.acquisitions import ExpectedImprovement
from emukit.core import ParameterSpace, ContinuousParameter, DiscreteParameter
from runner import call_sim
import matplotlib.pyplot as plt

class emu():
    def __init__(self, iterations):
        self.X = np.array([[1, 1, 1, 1, 0.5, 0.5, 1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]])
        self.Y = np.array([[call_sim(self.X[0])]])

        # Define parameter space for the simulator variables
        space = ParameterSpace([ContinuousParameter('traffic_light_1', 1, 10),
                                ContinuousParameter('traffic_light_2', 1, 10),
                                ContinuousParameter('traffic_light_3', 1, 10),
                                ContinuousParameter('traffic_light_4', 1, 10),
                                ContinuousParameter('sigma', 0.5, 0.5),
                                ContinuousParameter('tau', 0.5, 0.5), # Don't know how to do no upper bound
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
        gpy_model = GPy.models.GPRegression(self.X, self.Y, kern, noise_var=1e-10)
        # Emukit Model
        emukit_model = GPyModelWrapper(gpy_model)

        ################################################################################
        # OPTION 1: Manual
        ################################################################################

        # Acquisition function
        us_acquisition = ModelVariance(emukit_model)
        # Acquisition optimiser
        optimizer = GradientAcquisitionOptimizer(space)

        for _ in range(iterations):
            # Get next point
            global x_new
            x_new, _ = optimizer.optimize(us_acquisition)

            # Run simulator on new
            y_new = np.array([call_sim(list(x_new[0]))])


            self.X = np.append(self.X, x_new, axis=0)
            self.Y = np.append(self.Y, np.expand_dims(y_new, 0), axis=0)
            #print(f"X: {self.X}")
            #print(f"Y: {self.Y}")

            # Add data to model
            emukit_model.set_data(np.array(self.X), np.array(self.Y))

        self.model = emukit_model

        ################################################################################
        # OPTION 2: Easy
        ################################################################################

        # Basically zero documentation :(
        #from emukit.experimental_design.experimental_design_loop import ExperimentalDesignLoop
        #ed = ExperimentalDesignLoop(space=space,
        #                            model=emukit_model,
        #                            acquisition = ModelVariance(emukit_model),
        #                            acquisition_optimizer = GradientAcquisitionOptimizer(space)
        #                            )
        #ed.run_loop(target_function, 10)

        #self.model = ed.model

    def call(self, x):
        # Returns tuple of mean and variance
        return self.model.predict(x)

    # partial_x will be things like number of cars on the road
    def optimise(self, partial_x, iterations):
        assert len(partial_x) == 17

        # optimise_x are the parameters to optimise such as speed limit
        def partial_call(optimise_x):
            return self.call(np.expand_dims(np.append(optimise_x, partial_x), axis=0))[0]

        x_init = np.array([[1,1,1,1]])
        y_init = partial_call(x_init)

        optimise_x = [ContinuousParameter('traffic_light_1', 1, 100),
                      ContinuousParameter('traffic_light_2', 1, 100),
                      ContinuousParameter('traffic_light_3', 1, 100),
                      ContinuousParameter('traffic_light_4', 1, 100)]
        bo = GPBayesianOptimization(variables_list=optimise_x,
                            X=x_init, Y=y_init)
        bo.run_optimization(partial_call, iterations)

        print(bo.loop_state.X)
        print(bo.loop_state.Y)
        maximum = bo.loop_state.X[np.argmax(bo.loop_state.Y)]
        return maximum

    def new_optimise(self, partial_x):
        assert len(partial_x) == 17

        # Four dimensional GP for modelling changes in traffic lights for partial_x
        gpy_model = GPy.models.GPRegression(None, None, GPy.kern.RBF(4, lengthscale=0.08, variance=20), noise_var=1e-10)
        emukit_model = GPyModelWrapper(gpy_model)

        aq = ExpectedImprovement(emukit_model)
        space = ParameterSpace([ContinuousParameter('traffic_light_1', 1, 100),
                                ContinuousParameter('traffic_light_2', 1, 100),
                                ContinuousParameter('traffic_light_3', 1, 100),
                                ContinuousParameter('traffic_light_4', 1, 100),
                                ])
        optimizer = GradientAcquisitionOptimizer(space)

e = emu(20)
#x_plt = np.linspace(0,100,1000)[:, None]
#rest = np.array([[1,1,1,0.5,0.5,1,1,1,0.9,0.9,0.9,0.9,0.9,0.45,0.45,0.45,0.1,0.1,0.1,0.1]]*1000)
#y_plt, y_var = e.call(np.append(x_plt, rest, axis=1))
#plt.fill_between(x_plt[:,0], y_plt[:,0]+y_var[:,0], y_plt[:,0]-y_var[:,0], alpha=0.3)
#plt.plot(x_plt[:,0], y_plt[:,0])
#plt.xlim((0,100))
#plt.ylim((-50,50))
#plt.show()
print(e.optimise([0.5, 0.5, 1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], iterations = 100))
