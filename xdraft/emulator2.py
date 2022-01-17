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

train = True
if train:
    space = ParameterSpace([ContinuousParameter('traffic_light_1', 1, 4),
                                    ContinuousParameter('traffic_light_2', 1, 4),
                                    ContinuousParameter('traffic_light_3', 1, 4),
                                    ContinuousParameter('traffic_light_4', 1, 4),
                                    ContinuousParameter('trucks', 1, 1),
                                    ContinuousParameter('cars', 1, 2),
                                    ContinuousParameter('bikes', 1, 1),
                                    ContinuousParameter('Heaviness',0,1),
                                    ContinuousParameter('NB', 0.1, 0.5),
                                    ContinuousParameter('EB', 0.1, 0.5),
                                    ContinuousParameter('SB', 0.1, 0.5),
                                    ContinuousParameter('WB', 0.1, 0.5)
                                    ])
    num_data_points = 30
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
    optimizer        = GradientAcquisitionOptimizer(space = space)
    expdesign_loop = ExperimentalDesignLoop(space = space,
                                            model = model_emukit,
                                            acquisition = model_variance,
                                            update_interval = 1,
                                            batch_size = 5)


    # Run the loop
    stopping_condition = FixedIterationsStoppingCondition(i_max = 250)
    expdesign_loop.run_loop(f, stopping_condition)


    with open('model.pkl','wb') as pw:
        pickle.dump(model_emukit,pw)
        print("Dumped model succesfully")

else:
    with open('model.pkl','rb') as pw:
        model_emukit = pickle.load(pw)
        print("loaded model")



#Get some sample vals for plotting how it does
#For all these, assume everything bar traffic light #1 is fixed, i.e.:

fixed_params = [3,3,3,0.5,0.5, 1, 2, 1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
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
#real_ys = runner.call_sim_parallel(xs)

plt.title('Learning function sin(x) with Emukit')
plt.xlabel('x')
plt.ylabel('y', rotation=None)
xs = list(map(lambda x: x/4, range(1,44,1)))
#plt.plot(xs, real_ys, c='r', )
plt.plot(xs, predicted_y)
plt.legend(['True function', 'Estimated function'], loc='lower right')
plt.fill_between(xs, predicted_y - 2 * predicted_std, predicted_y + 2 * predicted_std, alpha=.5)
plt.show()

if __name__ == "__main__":
    print('fish')