# Installation
 
## For Windows & Ubuntu 18.04 and Earlier
* Pre-requisites
   * python3.7
   * carla (API)
   * pip 
   * numpy
   * pygame 
   * gtest 
* Help
   * sudo apt install python3.7
   * python3.7 -m pip install -U pip carla numpy pygame --user
   * sudo apt-get install libgtest-dev

   
* The deb installation is the easiest way to get the latest release in Linux.
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
sudo apt-get update # Update the Debian package index
sudo apt-get install carla-simulator=0.9.10-2 
cd /opt/carla-simulator
# Linux
./CarlaUE4.sh
```

More details can be found at [CARLA Quick Start Installation](https://carla.readthedocs.io/en/latest/start_quickstart/)  
> NOTE: CARLA does NOT work on Ubuntu 20.04 or Mac yet!!
 
## Download Udacity Project code-base    
* Once CARLA launches without problems you will clone the project code-base
   ```
   mkdir sdcnd_c5 # Or wherever you want to place
   cd sdcnd_c5
   git clone https://github.com/udacity/nd013-c5-planning-refresh.git
   ```
 
* Make sure ALL is working well:
   * Open a terminal an launch CARLA (if you don't have it open already):
   ```
   cd /opt/carla-simulator
   # Linux
   ./CarlaUE4.sh
   ```   
   * Open another terminal and launch Udacity’s Decision making & Motion Planning application:
 
   ```
   cd sdcnd_c5/nd013-c5-planning-refresh/project/solution_cubic_spirals_STARTER
   make run
   ```
* Follow the TODO’s on the project plan and code all your changes.
 
> NOTE:
>
> To just compile use:
> ```
> make
> ```
> To compile and run use:
> ```
> make run
> ``` 
> To just run the latest compiled project use
> ```
> make run.only
> ```
> 