import GPy
from emukit.model_wrappers.gpy_model_wrappers import GPyModelWrapper
from emukit.experimental_design.acquisitions import ModelVariance
from emukit.core.optimization import GradientAcquisitionOptimizer
from emukit.examples.gp_bayesian_optimization.single_objective_bayesian_optimization import GPBayesianOptimization

class emu():
    def __init__(self, iterations):
        X = None #TODO: Add some initial points
        Y = None #TODO

        # Define parameter space
        space = None #TODO (emukit.core.parameter_space.ParameterSpace)

        # Kernel
        kern = GPy.kern.RBF(1, lengthscale=0.08, variance=20)
        # GP
        gpy_model = GPy.models.GPRegression(X, Y, kern, noise_var=1e-10)
        # Emukit Model
        emukit_model = GPyModelWrapper(gpy_model)

        ################################################################################
        # OPTION 1: Manual
        ################################################################################

        # Acquisition function
        us_acquisition = ModelVariance(emukit_model)
        # Acquisition optimiser
        optimizer = GradientAcquisitionOptimizer(space)

        for _ in range(ITERATIONS):
            # Get next point
            x_new, _ = optimizer.optimize(us_acquisition)

            # Run simulator on new
            # TODO: y_new = Simulator(x_new)

            X = np.append(X, x_new, axis=0)
            Y = np.append(Y, y_new, axis=0)

            # Add data to model
            emukit_model.set_data(X, Y)

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
    def optimise(self, partial_x):

        # optimise_x are the parameters to optimise such as speed limit
        def partial_call(optimise_x):
            return self.call(partial_x+optimise_x)[0]

        # So there is now a second GP modelling the first one: is that what we want?
        optimise_x = None # TODO: list of variables we want to optimise.
        bo = GPBayesianOptimization(variables_list=optimise_x,
                            X=X_init, Y=Y_init)
        bo.run_optimization(partial_call, 10)
        maximum = None # TODO
        return maximum
