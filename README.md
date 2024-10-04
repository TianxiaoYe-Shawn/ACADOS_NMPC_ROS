# Trajectory Tracking using ACADOS and ROS

https://github.com/user-attachments/assets/89e89b3c-f219-4338-b196-8f4bde3d70c3


## Install and Compile ACACOS

following this [tutorial](https://docs.acados.org/installation/index.html) (choose **CMake** part to proceed, not **Make**) to install

If your Ubuntu is running on WSL, follow the **Windows 10+(WSL)** part first

## Running Example Interface

There are 3 interfaces of ACADOS:

1. [C Interface](https://docs.acados.org/c_interface/index.html)
2. [Python Interface](https://docs.acados.org/python_interface/index.html)
3. [Matlab + Simulink and Octave interface](https://docs.acados.org/matlab_octave_interface/index.html)

I choose Python Interface (Currently, Python >= 3.8 is tested).

1. Make sure you compile and install `acados` by following the [CMake installation instructions](https://docs.acados.org/installation/index.html).
2. In one directory you choose, create a Python virtual environment
    
    ```bash
    virtualenv env --python=/usr/bin/python3
    ```
    
3. activate this environment
    
    ```bash
    source env/bin/activate
    ```
    
4. Install `acados_template` Python package (replace the <acados_root> with the path to your acados ):
    
    ```bash
    pip install -e <acados_root>/interfaces/acados_template
    ```
    
5. Add the path to the compiled shared libraries `libacados.so, libblasfeo.so, libhpipm.so` to `LD_LIBRARY_PATH` (default path is `<acados_root/lib>`) by running:
    
    ```bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
    export ACADOS_SOURCE_DIR="<acados_root>"
    ```
    
6. Now (2024/6/25) ACADOS has a bug with the HPIPM & BLASFEO targets, so do the following to fix it:
    
    ```bash
    cd <acados_dir>
    rm build/* -rf
    cd build
    cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=ON -DHPIPM_TARGET=GENERIC -DBLASFEO_TARGET=GENERIC
    make -j4
    make install -j4
    # run a C example, e.g.:
    ./examples/c/sim_wt_model_nx6
    ```
    
    if you can successfully run a C example, meaning you has fixed this bug
    
7. Now run a Python example:
    
    ```bash
    cd <acados_dir>
    cd examples/acados_python/getting_started/
    python3 minimal_example_ocp.py
    ```
    
    you will see bug: `! LaTeX Error: File 'type1cm.sty' not found.`
    
    Now install texlive and its dependencies

   ```bash
   sudo apt install texlive
   sudo apt install texlive-latex-extra cm-super dvipng
   ```
    
    Then run it again:
    
    ```bash
    python3 minimal_example_ocp.py
    ```
    
    You should see the result like the following:
    
    ![Untitled](https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/image.png)
    
    Means you have successfully installed ACAODS

## Solver Design and Simulation using ACADOS Python Interface
*First make sure you have successfully [installed ACADOS](https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/README.md#install-acados)*

In this robot model, system input is `u = [v, w]` where v denotes the linear velocity of the vehicle and w denotes the vehicle's angular velocity. 

System state is `X = [x, y, theta]` where x and y denote to the x y coordinates of the vehicle's position on the two-dimensional plane and theta denotes to the angle between the vehicle's direction and the positive x-axis at two adjacent x and y coordinates.

Below are simulations with three different point counts: a total of `100` points, a total of `300` points, and a total of `500` points. 

The initial states for these three simulations is the same with `X0 = [-2.0, 5.3, -0.00629628]`.

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_100_plot.png" alt="First Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_100_points.png" alt="First Image Bottom" width="100%" />
      <br>Simulation with 100 points
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_plot.png" alt="Second Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_points.png" alt="Second Image Bottom" width="100%" />
      <br>Simulation with 300 points
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_500_plot.png" alt="Third Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_500_points.png" alt="Third Image Bottom" width="100%" />
      <br>Simulation with 500 points
    </td>
  </tr>
</table>

Below are simulations starting from three different initial states: `X0 = [-2.0, 5.3, -0.00629628]`, `X0 = [-1.0, 3.0, -0.00629628]`, and `X0 = [1.0, 5.0, 1.57629628]`, respectively. The number of points for each simulation is the same, with a total of `300` points.

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_plot.png" alt="First Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_points.png" alt="First Image Bottom" width="100%" />
      <br>Simulation with X0 = [-2.0, 5.3, -0.00629628]
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_plot_2.png" alt="Second Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_points_2.png" alt="Second Image Bottom" width="100%" />
      <br>Simulation with X0 = [-1.0, 3.0, -0.00629628]
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_plot_3.png" alt="Third Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/blob/master/blob/master/sim_300_points_3.png" alt="Third Image Bottom" width="100%" />
      <br>Simulation with X0 = [1.0, 5.0, 1.57629628]
    </td>
  </tr>
</table>

To run the simulation in your own workspace:

In `robot_model.py` modify your robot model, in `create_solver.py` modify your ocp definition, in `solver.py` design your solver, then run:
```
cd /path/to/scripts/
source /path/to/your/virtualenv/bin/activate
python3 main.py
```


## Solver Test using ACADOS C Interface

## Combine C code with ROS nodes and simulate in Gazebo
