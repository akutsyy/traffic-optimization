from GPy.models import GPRegression
from emukit.core.initial_designs import RandomDesign
from emukit.core.loop import FixedIterationsStoppingCondition
from emukit.core.optimization import GradientAcquisitionOptimizer
from emukit.experimental_design.acquisitions import ModelVariance
from emukit.experimental_design.experimental_design_loop import ExperimentalDesignLoop
from emukit.core import ParameterSpace, ContinuousParameter
import GPy
from emukit.model_wrappers import GPyModelWrapper
import runner
import numpy as np
import matplotlib.pyplot as plt
import pickle

def train_model(max_iterations=250):
    space = ParameterSpace([ContinuousParameter('traffic_light_1', 1, 10),
                                    ContinuousParameter('traffic_light_2', 1, 10),
                                    ContinuousParameter('traffic_light_3', 1, 10),
                                    ContinuousParameter('traffic_light_4', 1, 10),
                                    ContinuousParameter('sigma', 0.5, 0.5),
                                    ContinuousParameter('tau', 0.5, 0.5),
                                    ContinuousParameter('trucks', 1, 1),
                                    ContinuousParameter('cars', 2, 2),
                                    ContinuousParameter('bikes', 1, 1),
                                    ContinuousParameter('NE', 0.1, 0.5),
                                    ContinuousParameter('NS', 0.1, 0.5),
                                    ContinuousParameter('NW', 0.1, 0.5),
                                    ContinuousParameter('EN', 0.1, 0.5),
                                    ContinuousParameter('ES', 0.1, 0.5),
                                    ContinuousParameter('EW', 0.1, 0.5),
                                    ContinuousParameter('SN', 0.1, 0.5),
                                    ContinuousParameter('SE', 0.1, 0.5),
                                    ContinuousParameter('SW', 0.1, 0.5),
                                    ContinuousParameter('WN', 0.1, 0.5),
                                    ContinuousParameter('WE', 0.1, 0.5),
                                    ContinuousParameter('WS', 0.1, 0.5),
                                    ])
    num_data_points = 5
    f = runner.call_sim_parallel
    design = RandomDesign(space)
    X = design.get_samples(num_data_points)
    print(X)
    Y = np.array(runner.call_sim_parallel(X))
    print(Y)
    kern = GPy.kern.RBF(21, lengthscale=0.08, variance=20)
    model_gpy = GPRegression(X,Y, kern, noise_var=1e-10)
    model_gpy.optimize()
    model_emukit = GPyModelWrapper(model_gpy)
    model_variance   = ModelVariance(model = model_emukit)
    optimizer = GradientAcquisitionOptimizer(space = space)
    expdesign_loop = ExperimentalDesignLoop(space = space,
                                            model = model_emukit,
                                            acquisition = model_variance,
                                            update_interval = 1,
                                            batch_size = 5)


    # Run the loop
    expdesign_loop.run_loop(f, 100)

    with open('model.pkl','wb') as pw:
        pickle.dump(model_emukit,pw)
        print("Dumped model succesfully")


if __name__ == '__main__':
    train = True
    if train:
        train_model()


    with open('model.pkl','rb') as pw:
        model_emukit = pickle.load(pw)
        print("loaded model")



    #Get some sample vals for plotting how it does
    #For all these, assume everything bar traffic light #1 is fixed, i.e.:

    fixed_params = [3,3,3,0.5, 0.5, 1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #Then pick values for the 4 traffic light times:
    xs = []
    for i in range(1,44,1):
        k = fixed_params.copy()
        k.insert(0,i/4)
        xs.append(k)

    print(xs)

    predicted_y = []
    predicted_std = []
    for x in xs:
        y, var = model_emukit.predict(np.array([x]))
        std = np.sqrt(var)
        predicted_y.append(y)
        predicted_std.append(std)

    predicted_y = np.array(predicted_y).flatten()
    print(predicted_y)
    predicted_std = np.array(predicted_std).flatten()
    real_ys = runner.call_sim_parallel(xs)

    plt.title('Learning traffic light timing for light 1')
    plt.xlabel('x')
    plt.ylabel('y', rotation=None)
    xs = list(map(lambda x: x/4, range(1,44,1)))
    plt.plot(xs, real_ys, c='r', )
    plt.plot(xs, predicted_y)
    plt.legend(['True function', 'Estimated function'], loc='lower right')
    plt.fill_between(xs, predicted_y - 2 * predicted_std, predicted_y + 2 * predicted_std, alpha=.5)
    plt.show()
