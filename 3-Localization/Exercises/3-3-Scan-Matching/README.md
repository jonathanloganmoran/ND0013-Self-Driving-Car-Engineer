# Lesson 3.3: Scan Matching Algorithms
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

## Objectives
* Use and fine-tune the [Point Cloud Library](https://en.wikipedia.org/wiki/Point_Cloud_Library) (PCL) to implement the [Iterative Closest Point](https://en.wikipedia.org/wiki/Iterative_closest_point) (ICP) algorithm in C++;
* Use the [KD Tree](https://en.wikipedia.org/wiki/K-d_tree) with [nearest neighbour search](https://en.wikipedia.org/wiki/Nearest_neighbor_search) to register points between the two point clouds;
* Use [Newton's method](https://en.wikipedia.org/wiki/Newton%27s_method) and [Singular Value Decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) (SVD) to implement the [Normal Distributions Transform](https://en.wikipedia.org/wiki/Normal_distributions_transform) (NDT) algorithm in C++ and recover the transformation matrix.


## 1. Introduction
In this lesson we implement two popular scan matching algorithms for [point-set registration](https://en.wikipedia.org/wiki/Point-set_registration). Using the [Iterative Closest Point](https://en.wikipedia.org/wiki/Iterative_closest_point) (ICP) and [Normal Distributions Transform](https://en.wikipedia.org/wiki/Normal_distributions_transform) (NDT) algorithms we perform the registration and recover the translation / rotation between the input `source` and `target` point clouds. In the following three exercises we write a variety of functions fundamental to the local localisation task in the 2D setting. Here we used simulated LiDAR point cloud data generated with the Point Cloud Library (PCL) ray-tracing functions. We apply our NDT and ICP algorithms to the simulated data and estimate a transformation that can be visualised using the Point Cloud Library [PCL Viewer](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcl_visualizer.html) instance. Let's get started...


## 2. Programming Task

### 2.1. Introduction to Iterative Closest Point (ICP) 

#### Fine-tuning the ICP algorithm

In this exercise we implement the 2D Iterative Closest Point (ICP) algorithm using the standard [`pcl::IterativeClosestPoint`](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html) PCL library class. In order to fine-tune the registration, we modify the following hyperparameters in [`icp1-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/icp1-main.cpp):
* `kMaxCorrespondenceDistanceICP`: The maximum correspondence distance between `source` and `target` point clouds. Correspondences with a distance larger than this threshold will be ignored during registration. Should be sufficiently large such that all points in both point clouds are considered. Rule of thumb: set to the maximum distance between two points in the point clouds; 
* `kMaximumIterationsICP`: the maximum number of ICP iterations to perform before termination. Should be large enough to ensure the algorithm has sufficient time to converge. Rule of thumb: set to twice the number of total points in the point cloud;
* `kTransformationEpsilonICP`: Maximum epsilon threshold between the previous transformation and the current estimated transformation. Rule of thumb: set to a value between `1e-04` and `1e-08`;
* `kEuclideanFitnessEpsilonICP`: Maximum allowed sum of Euclidean squared errors between two consecutive steps before the algorithm is considered to have converged. Rule of thumb: set to a value between `1` and `10`;
* `kRANSACOutlierRejectionThresholdICP`: the inlier distance threshold for the internal [Random sample consensus](https://en.wikipedia.org/wiki/Random_sample_consensus) (RANSAC) outlier rejection algorithm. A point is considered an inlier if the distance between the `target` and transformed `source` is smaller than this inlier distance threshold. By default, this value is set to `0.05m`. Rule of thumb: set to a value between `0.2` and `0.3` metres.

These hyperparameters are also known as the termination criteria for the ICP algorithm. More details on their implementation and how to set / retrieve these values, see the [`pcl::registration.h`](https://pointclouds.org/documentation/registration_2include_2pcl_2registration_2registration_8h_source.html) file. Recommendations for the values of these hyperparameters were obtained in a conversation with GPT model `text-davinci-003` [here](https://beta.openai.com/playground/p/SlZwcXmHPzNSrgFP2kTGy87N?model=text-davinci-003).


#### Setting the programme execution flags

In order to run the various parts of this exercise, you must set the corresponding execution flags inside the [`icp1-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/icp1-main.cpp) file.

##### Part One: Registration over single robot step
To run Part One of the ICP programme, update the execution flags with the following values:

```cpp
// Perform Step 2 : Localisation with ICP after several time-steps
const static bool runPart2 = false;
// Perform Step 3 : Localisation with ICP after complete random walk 
const static bool runPart3 = false;
```

Setting the above flags to `false` will configure the programme to only simulate a single robot move. The ICP scan matching algorithm is therefore performed on only one set of `source` and `target` point clouds.

##### Part Two: Registration over several robot steps
To run Part Two of the ICP programme, update the execution flags with the following values:

```cpp
// Perform Step 2 : Localisation with ICP after several time-steps
const static bool runPart2 = true;
// Perform Step 3 : Localisation with ICP after complete random walk 
const static bool runPart3 = false;
```

This will configure the programme to simulate three robot moves. The ICP scan matching and localisation algorithm will therefore return a set of three transformation matrix estimations along with three pose estimates. Here's what you should expect to see:


<img src="out/2022-12-01-Output-1-ICP-Localisation-After-3-Steps.png" width="80%" height="80%" alt="Figure 1. Output 1 from the `icp1-main.cpp` programme — pose and transformation estimates after three simulated robot moves.">
$$
\begin{align}
\textrm{Figure 1. Output 1 from the `icp1-main.cpp` programme — pose and transformation estimates after three simulated robot moves.}
\end{align}
$$

The pose estimates are shown in green, while the ground-truth robot poses are shown in blue. The ground-truth transformations are shown in red.  


##### Part Three: Registration over a random walk

To run Part Three of the ICP programme, update the execution flags with the following values:

```cpp
// Perform Step 2 : Localisation with ICP after several time-steps
const static bool runPart2 = true;
// Perform Step 3 : Localisation with ICP after complete random walk 
const static bool runPart3 = true;
```

This will configure the programme to simulate a random walk of robot moves around the 2D environment. The ICP scan matching and localisation algorithm should produce a corresponding set of pose and transformation estimates. You can expect to obtain the following:

<img src="out/2022-12-01-Output-4-ICP-Localisation-After-Random-Walk-Before-After.png" width="80%" height="80%" alt="Figure 2. Output 4 from the `icp1-main.cpp` programme — pose and transformation estimates before (left) and after (right) ICP after a random walk of simulated robot moves.">
$$
\begin{align}
\textrm{Figure 2. Output 4 from the `icp1-main.cpp` programme — pose and transformation estimates before (left) and after (right) ICP after a random walk of simulated robot moves.}
\end{align}
$$

In the above figure we see that the robot is localised with the ICP algorithm as indicated on the right-hand side of the figure. Shown in green are the estimated pose and transformations after performing the ICP algorithm, while the ground-truth robot poses are shown in blue. The robot's ground-truth transformations at each time-step are shown in red.

#### Running and compiling the programme

If running in the Udacity VM environment on Linux Ubuntu 18.04.5 LTS, make sure the **GPU is enabled**. This is not only important for Point Cloud Library (PCL) performance but also in order to make sure the appropriate C++ libraries are made available at runtime.

##### Configuring `CMake`
You must compile the programme before it is executable. To do so, first configure line 14 of the [`CMakeLists.txt`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/CMakeLists.txt) file. Set the line to the following:

```cpp
// In CMakeLists.txt:
set(sources {FILENAME OF MAIN} {FILENAME OF HELPERS})
```
where `{FILENAME OF MAIN}` should be [`icp1-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3-3-Scan-Matching/icp1-main.cpp), and `{FILENAME OF HELPERS}` should be [`helpers.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/helpers.cpp).

Note that the [`helpers.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3-3-Scan-Matching/helpers.cpp) and [`helpers.h`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/helpers.h) files contain signficant modifications to the original files provided by Udacity. These modifications were added to allow the compatibility of various functions and classes across the three Exercise files.

##### Creating the executable

To create the executable file, first make sure that you have created a `build` folder inside the project directory. Then, navigate to the `build` folder and execute the following command:

```console
root@foobar:/home/workspace/build/# cmake ..
```

If on the Udacity VM, this will initialise the build script located in the project root directory at the path `/home/workspace/`. 

Now, from inside the `build` folder, run:
```console
root@foobar:/home/workspace/build/# make
```
to build the programme files.

Once compiled succesfully, you should see in the `build` folder an object file named `icp.o`. To run the programme, execute the following:

```console
root@foobar:/home/workspace/build/# ./icp
```

Note that if running on the Udacity VM, any PCL visualisations will appear only the remote desktop; if you work in the workspace IDE you will need to click on the "Desktop" button in the bottom right, and only run the executable from the terminal within the remote desktop to view them.


### 2.2. Creating the Iterative Closest Point (ICP) Algorithm

In this Exercise 3.2 we are writing the ICP algorithm from scratch. In order to implement the [point-set registration](https://en.wikipedia.org/wiki/Point-set_registration) we use the [KD Tree](https://en.wikipedia.org/wiki/K-d_tree) data structure to store the centred vectors of the `target` point cloud. In order to minimise the computations required to perform `source` to `target` point association, we run a [nearest neighbours](https://en.wikipedia.org/wiki/Nearest_neighbor_search) radius search over the `target` points for each point index in `source`. Using an arbitrary radius `dist` for the query points of `source`, we obtain an efficient retrieval algorithm which performs nearest neighbour matching in log-n time. We use a centred vectors approach, i.e., a set of `target` points centred around a `source` origin point, in order to make the distance comparison easier between the two point clouds. In other words, the origin `source` point is used as a reference point and the KD Tree search explores only the candidate `target` points neighbouring the given `source` point within a specific radius. The use of this approach makes the search faster and more efficient by minimising the number of comparisons from all points to only those within a specified radius (ref: ). To do so, we use the PCL [`pcl::KdTreeFLANN`](https://pointclouds.org/documentation/classpcl_1_1_kd_tree_f_l_a_n_n.html) class. A tutorial on how to use this PCL library function can be found [here](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html).

Once the pairs are computed, we estimate the rotation and translation parameters in an registration using the Iterative Closest Point (ICP) algorithm. In the `ICP` function of [`icp2-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/icp2-main.cpp) we compute the transformation of the `source` point cloud using the estimated nearest neighbour pairs obtained from the above `PairPoints` with the KD Tree search. In order to recover the transformation matrix between these point pairs, we compute the matrix decomposition using the [Singular Value Decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) (SVD) algorithm. We use the SVD algorithm here to solve the optimal alignment problem because it allows for the decomposition of the transformation matrix into its constituent singular values and vectors. Here those singular values represent the magnitude of the differences in position between the two point sets, while the vectors represent the differences in orientation (direction) between the two point sets. Using the singular values and vectors, we can estimate the original transformation matrix between the `source` and `target` point sets and therefore obtain the optimal alignment (ref: [here](https://beta.openai.com/playground/p/7znB72LQ9sVXFyohkJReZeHO?model=text-davinci-003)]).

The SVD algorithm used in this programme has been referenced directly from the Sorokine-Hornung et al., 2017 paper [1].


#### Running and compiling the programme

If running in the Udacity VM environment on Linux Ubuntu 18.04.5 LTS, make sure the **GPU is enabled**. This is not only important for Point Cloud Library (PCL) performance but also in order to make sure the appropriate C++ libraries are made available at runtime.


##### Part One: Manual Alignment With Iterative Closest Point (ICP)
In order to run this Exercise 3.2, no configuration is needed of the [`icp2-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/icp2-main.cpp) file. Simply update the CMake config file and run the build steps as follows. Once built successfully, you can perform manual alignment of the point clouds interatively using the keyboard input commands in Part Three of Exercise 3.1. In other words, you may shift / translate and rotate the estimated transformation using the `k`, `l` keys and left-arrow, right-arrow keys of your keyboard. You will see the updated alignment score reported for each movement of the "estimated" transformation, as demonstrated below:

<img src="out/2022-12-04-Output-1-Manual-Alignment.gif" width="80%" height="80%" alt="Figure 3. Output 1 from the `icp2-main.cpp` programme — manual pose and transformation alignment with current error and ICP-computed error scores.">
$$
\begin{align}
\textrm{Figure 3. Output 1 from the `icp2-main.cpp` programme — manual pose and transformation alignment with current error and ICP-computed error scores.}
\end{align}
$$

In the above animated GIF we see that the ICP score is computed for each change in the rotation / translation entered by the user. We observe that the true error `Score` reported by each transformation closely matches the estimated `ICP Score` computed after the point-set registration has been performed.


##### Configuring `CMake`
You must compile the programme before it is executable. To do so, first configure line 14 of the [`CMakeLists.txt`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/CMakeLists.txt) file. Set the line to the following:

```cpp
// In CMakeLists.txt:
set(sources {FILENAME OF MAIN} {FILENAME OF HELPERS})
```
where `{FILENAME OF MAIN}` should be [`icp2-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3-3-Scan-Matching/icp2-main.cpp), and `{FILENAME OF HELPERS}` should be [`helpers.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/helpers.cpp).

Note that the [`helpers.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3-3-Scan-Matching/helpers.cpp) and [`helpers.h`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/helpers.h) files contain signficant modifications to the original files provided by Udacity. These modifications were added to allow the compatibility of various functions and classes across the three Exercise files.

##### Creating the executable

To create the executable file, first make sure that you have created a `build` folder inside the project directory. Then, navigate to the `build` folder and execute the following command:

```console
root@foobar:/home/workspace/build/# cmake ..
```

If on the Udacity VM, this will initialise the build script located in the project root directory at the path `/home/workspace/`. 

Now, from inside the `build` folder, run:
```console
root@foobar:/home/workspace/build/# make
```
to build the programme files.

Once compiled succesfully, you should see in the `build` folder an object file named `icp.o`. To run the programme, execute the following:

```console
root@foobar:/home/workspace/build/# ./icp
```

Note that if running on the Udacity VM, any PCL visualisations will appear only the remote desktop; if you work in the workspace IDE you will need to click on the "Desktop" button in the bottom right, and only run the executable from the terminal within the remote desktop to view them.


### 2.3. Creating the Normal Distributions Transform (NDT) Algorithm

In this final Exercise 3.3 of Lesson 3.3 we implement the [Normal Distributions Transform](https://en.wikipedia.org/wiki/Normal_distributions_transform) (NDT) algorithm. Like the previous two ICP programmes, we implement the NDT to perform scan matching of the `source` and `target` point clouds. However, the NDT algorithm uses a very different approach to estimating the transformation (registration) between these two point clouds. By discretising the point cloud sets into individual `grid` instances, we are able to represent the distributions of points within each of the overlapping `Cells` making up each `grid` as a [normal distribution](https://en.wikipedia.org/wiki/Normal_distribution) parameterised by the point clusters' mean and covariance values. Using the [probability density function](https://en.wikipedia.org/wiki/Probability_density_function) (PDF), we can evaluate the probability of a given point residing inside the respective `grid` as the sum of individual probabilities calculated for each overlapping `Cell` within the `grid`. Assuming the [multi-variate case](https://en.wikipedia.org/wiki/Multivariate_normal_distribution), we use the [Newton's method](https://en.wikipedia.org/wiki/Newton%27s_method) to compute the roots of the error function relating the two point cloud sets. This non-linear error function is computed as the [Mahalanobis distance](https://en.wikipedia.org/wiki/Mahalanobis_distance), which is minimised using the Newton's method algorithm in order to estimate the parameters of the transformation matrix (i.e., the rotation, translation and scaling) between the two point clouds. By iteratively computing the second-order partial derivatives of this error function w.r.t. the transformation parameters (i.e., the Hessian), and the vector of first-order partial derivatives (i.e., the gradient), we can determine the direction of steepest descent towards the region of convergence along the non-linear error function. This results in a more accurate and quicker determination of the transformation parameters used to match the first `source` point cloud with the second `target` point cloud (ref: [here](https://beta.openai.com/playground/p/K5nm6To3fDhvDx6SneduIGv0?model=text-davinci-003)).

The NDT algorithm and its respective formulas implemented in [`ndt-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/ndt-main.cpp) were referenced explicitly from Biber et al., 2003 [2].

#### Running and compiling the programme

##### Part One: Visualising the Normal Distribution

In order to visualise the probability values obtained from the probability distribution function, we update the execution flags with the following:

```cpp
// Part 1 : Visualise the probabilities of a single `grid` computed with PDF
// If False, Part 2 is run over all `grid` instances in point cloud
const static bool kRunPart1 = true;
```

and set the following NDT hyperparameters to the indicated values:
```cpp
// CANDO: Increase the iteration count to get convergence
const static int kGradientIterations = 1;
```

By running the programme with the Part One flag set to `true`, and setting the gradient steps to just `1`, we should get an output similar to the following:

<img src="out/2022-12-06-Output-3-NDT-Probabilities-Side-in-2D-3D-View.png" width="80%" height="80%" alt="Figure 4. Output 1 from the `ndt-main.cpp` programme — probability values for each discretised position in a given `Cell` obtained from the PDF of a parameterised normal distribution.">
$$
\begin{align}
\textrm{Figure 4. Output 1 from the `ndt-main.cpp` programme — probability values for each discretised position in a given `Cell` obtained from the PDF of a parameterised normal distribution.}
\end{align}
$$

We see here the probability values of the normal distribution colourised as intensity values in the PCL Viewer instance. In the above we note that the centre of the distribution corresponds to the greatest probability as shown in blue. In the 3D view shown on the right-hand side, this depiction is more obvious.


##### Part Two: Manual Alignment with Normal Distributions Transform (NDT)

In order to run Part Two of the NDT programme, i.e., the interactive alignment, update the execution flags to the following:

```cpp
// Part 1 : Visualise the probabilities of a single `grid` computed with PDF
// If False, Part 2 is run over all `grid` instances in point cloud
const static bool kRunPart1 = false;
```

and set the NDT hyperparameters to a value greater than zero, i.e.,

```cpp
// CANDO: Increase the iteration count to get convergence
const static int kGradientIterations = 20;
```

In Part Two of the programme we are able to manually align the transformation between the two point clouds using the same key inputs as in Part 2 of Exercise 3.2. Likewise, we see the corresponding NDT alignment score estimated w.r.t. the user-inputted transformation.

<img src="out/2022-12-06-Output-2-NDT-Manual-Alignment.gif" width="80%" height="80%" alt="Figure 5. Output 2 from the `ndt-main.cpp` programme — manual alignment with corresponding NDT alignment score of the user-inputted transformation.">
$$
\begin{align}
\textrm{Figure 5. Output 2 from the `ndt-main.cpp` programme — manual alignment with corresponding NDT alignment score of the user-inputted transformation.}
\end{align}
$$

##### Configuring `CMake`
You must compile the programme before it is executable. To do so, first configure line 14 of the [`CMakeLists.txt`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/CMakeLists.txt) file. Set the line to the following:

```cpp
// In CMakeLists.txt:
set(sources {FILENAME OF MAIN} {FILENAME OF HELPERS})
```
where `{FILENAME OF MAIN}` should be [`ndt-main.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3-3-Scan-Matching/ndt-main.cpp), and `{FILENAME OF HELPERS}` should be [`helpers.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/helpers.cpp).

Note that the [`helpers.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/Exercises/3-3-Scan-Matching/helpers.cpp) and [`helpers.h`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3Exercises/-3-Scan-Matching/helpers.h) files contain signficant modifications to the original files provided by Udacity. These modifications were added to allow the compatibility of various functions and classes across the three Exercise files.

##### Creating the executable

To create the executable file, first make sure that you have created a `build` folder inside the project directory. Then, navigate to the `build` folder and execute the following command:

```console
root@foobar:/home/workspace/build/# cmake ..
```

If on the Udacity VM, this will initialise the build script located in the project root directory at the path `/home/workspace/`. 

Now, from inside the `build` folder, run:
```console
root@foobar:/home/workspace/build/# make
```
to build the programme files.

Once compiled succesfully, you should see in the `build` folder an object file named `ndt.o`. To run the programme, execute the following:

```console
root@foobar:/home/workspace/build/# ./ndt
```

Note that if running on the Udacity VM, any PCL visualisations will appear only the remote desktop; if you work in the workspace IDE you will need to click on the "Desktop" button in the bottom right, and only run the executable from the terminal within the remote desktop to view them.

This concludes the Exercises from Lesson 3.3: Creating Scan Matching Algorithms.

## 3. Closing Remarks
##### Alternatives
* Use other point-set registration algorithms to estimate both rigid- and non-rigid transformations (e.g., Umeyama Algorithm [3], others [here](https://beta.openai.com/playground/p/2Agta0bS5i20WVtUqwVtqOGi?model=text-davinci-003)).

##### Extensions of Task
* Experiment with different hyperparameter settings for the ICP algorithm (e.g., fine-tuning maximum ICP iterations, RANSAC inlier threshold — notes from GPT [here](https://beta.openai.com/playground/p/SlZwcXmHPzNSrgFP2kTGy87N?model=text-davinci-003));
* Experiment with different implementation settings for the NDT algorithm (e.g., increasing cell resolution, adjusting number of gradient iterations, changing step size — notes from GPT [here](https://beta.openai.com/playground/p/AGoz2dlj4SqUYm2xl2LYzM9T?model=text-davinci-003)).

## 4. Future Work
* ✅ Experiment with different ICP hyperparameters (e.g., fine-tune max correspondence distance and maximum ICP iterations to the point cloud dataset) — completed in [Project 3.1: Scan Matching Localization](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/3-Localization/3-1-Scan-Matching-Localization);
* ✅ Experiment with different NDT implementation settings (e.g., increasing cell resolution, adjusting gradient iterations) — completed in [Project 3.1: Scan Matching Localization](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/3-Localization/3-1-Scan-Matching-Localization).


## Credits
This course was prepared by A. Brown, T. Huang, and M. Muffert of the Mercedes-Benz Research and Development of North America (MBRDNA), 2021 (link [here](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013)).


References
* [1] Sorokine-Hornung, O. et al. Least-Squares Rigid Motion Using SVD. Department of Computer Science, ETH Zürich. 2017. [PDF](https://igl.ethz.ch/projects/ARAP/svd_rot.pdf).

* [2] Biber, P. et al. The normal distributions transfom: a new approach to laser scan matching. Proceedings of the 2003 IEEE/RSJ Conference on Intelligent Robots and Systems (IROS 2003) (Cat. No.03CH37453). 3(1):2743-2748. 2003. [doi:10.1109/IROS.2003.1249285](https://doi.org/10.1109/IROS.2003.1249285).

* [3] Umeyama, S. Least-Squares Estimation of Transformation Parameters Between Two Point Patterns. IEEE Transactions on Pattern Analysis and Machine Intelligence. 13(4):376-380. 1991. [doi:10.1109/34.88573](https://doi.ieeecomputersociety.org/10.1109/34.88573). 


Helpful resources:
* [](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html)