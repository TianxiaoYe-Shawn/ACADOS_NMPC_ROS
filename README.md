[`中文版教程在这`](https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/README.md#%E4%BB%8E%E9%9B%B6%E5%BC%80%E5%A7%8B%E6%90%AD%E5%BB%BA%E5%9F%BA%E4%BA%8Eacado%E7%9A%84%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AAros%E5%8A%9F%E8%83%BD%E5%8C%85)

# Trajectory Tracking using ACADOS and ROS

https://github.com/user-attachments/assets/d8c6dd66-8fd8-4ec9-8bb8-126d54ccae5c


## Install and Compile ACACOS

following this [tutorial](https://docs.acados.org/installation/index.html) (choose **CMake** part to proceed, not **Make**) to install

If your Ubuntu is running on WSL/WSL2, follow the **Windows 10+(WSL)** part first

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
    
    ![Untitled](https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/image.png)
    Means you have successfully installed ACAODS

## Solver Design and Simulation using ACADOS Python Interface
*First make sure you have successfully [installed ACADOS](https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/README.md#install-acados)*

In this robot model, system input is `u = [v, w]` where v denotes the linear velocity of the vehicle and w denotes the vehicle's angular velocity. 

System state is `X = [x, y, theta]` where x and y denote to the x y coordinates of the vehicle's position on the two-dimensional plane and theta denotes to the angle between the vehicle's direction and the positive x-axis at two adjacent x and y coordinates.

Below are simulations with three different point counts: a total of `100` points, a total of `300` points, and a total of `500` points. 

The initial states for these three simulations is the same with `X0 = [-2.0, 5.3, -0.00629628]`.

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_100_plot.png" alt="First Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_100_points.png" alt="First Image Bottom" width="100%" />
      <br>Simulation with 100 points
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_plot.png" alt="Second Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_points.png" alt="Second Image Bottom" width="100%" />
      <br>Simulation with 300 points
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_500_plot.png" alt="Third Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_500_points.png" alt="Third Image Bottom" width="100%" />
      <br>Simulation with 500 points
    </td>
  </tr>
</table>

Below are simulations starting from three different initial states: `X0 = [-2.0, 5.3, -0.00629628]`, `X0 = [-1.0, 3.0, -0.00629628]`, and `X0 = [1.0, 5.0, 1.57629628]`, respectively. The number of points for each simulation is the same, with a total of `300` points.

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_plot.png" alt="First Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_points.png" alt="First Image Bottom" width="100%" />
      <br>Simulation with X0 = [-2.0, 5.3, -0.00629628]
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_plot_2.png" alt="Second Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_points_2.png" alt="Second Image Bottom" width="100%" />
      <br>Simulation with X0 = [-1.0, 3.0, -0.00629628]
    </td>
    <td align="center">
      <img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_plot_3.png" alt="Third Image Top" width="100%" />
      <br><img src="https://github.com/TianxiaoYe-Shawn/ACADOS_NMPC_ROS/blob/master/sim_300_points_3.png" alt="Third Image Bottom" width="100%" />
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

# 从零开始搭建基于ACADO的轨迹跟踪ROS功能包

## 1.Ubuntu和ROS版本

本ros功能包构建在Ubuntu20.04和ROS Noetic桌面完全版上。建议按照这个版本来，后续的JACKAL模拟器也是在这个版本上构建的。

## 2.创建ROS工作空间

复制以下代码到终端以创建工作空间：

```
mkdir -p ~/NMPC_ACADO_ws/src/
cd ~/NMPC_ACADO_ws/src/
catkin_init_workspace
cd ~/NMPC_ACADO_ws
catkin_make
echo "source ~/NMPC_ACADO_ws/devel/setup.bash" >> ~/.bashrc
```

你可以将`NMPC_ACADO_ws`修改成你定义的工作空间名。

## 3.下载ROS功能包

进入你工作空间下的`/src`目录，下载功能包：

```
cd ~/NMPC_ACADO_ws/src
git clone https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS.git
```

## 4.安装ACADO

安装依赖：

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
```

任意选择一个工作目录（我是在`/home/~/`）下载ACADO源码：

```
cd
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
```

安装：

```
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
sudo make install
```

配置环境变量：

```
echo "source ~/ACADOtoolkit/build/acado_env.sh" >> ~/.bashrc
```

## 5.编译ROS功能包

ACADO的优点就是能够通过符号化的语言生成高效的c代码。首先在`acado_mpc/acado_export_code`中的`symbolic_mpc.cpp`文件修改你自己的mpc模型（第一次跑建议不要动）。

然后生成c代码包：

```
cd ~/NMPC_ACADO_ws/src/acado_mpc/acado_export_code
mkdir build && cd build
cmake ..
make
./mpc
```

移动生成的代码并搭建静态库

```
mv symbolic_mpc_export/* ../../acado_mpc_export/
cd ../../acado_mpc_export
make
```

添加 `include` 文件夹（存放头文件的，目前是空的，你可以把你未来写的头文件放进去）:
```
cd ~/NMPC_ACADO_ws/src/acado_mpc/
mkdir include
```

编译整个ros功能包

```
cd ~/NMPC_ACADO_ws
catkin_make
```

如果一切成功，至此你的功能包就已经搭建完毕了。如果你没有任何的测试环境，接下来这个JACKAL模拟器是不错的选择。

## 6.搭建JACKAL模拟器

一键安装JACKAL小车模拟器

```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```

修改模拟器的环境，我们需要一个空旷的场地

```
cd /opt/ros/noetic/share/jackal_gazebo/launch/
sudo vi jackal_world.launch
```

按`i`进入编辑模式

将`<arg name="world_name..."`那句修改成 `<arg name="world_name" default="$(find gazebo_ros)/launch/empty_world.launch" />`

我们还需要调整小车初始位姿与轨迹开头重合

将 `x` `y` `z` `yaw` 的值修改成 `0 0 1.0 0.78`

然后按`ESC`输入`:wq`保存并退出。

至此JACKAL模拟器搭建完毕。

## 7.开始运行！

（以下每一个`rosrun`和`roslaunch`都需要你重新打开一个新的终端）

首先打开JACKAL模拟器：

```
roslaunch jackal_gazebo jackal_world.launch
```

你应该能看到JACKAL小车在gazebo里停着。

然后打开轨迹跟踪的测试环境（包括发布轨迹，配置rviz）：

```
roslaunch acado_mpc tracking_env.launch
```

你应该能看到rviz中有绿色圆形轨迹（你可以在`trajectory_publisher.cpp`文件中自定义你的轨迹）。

然后打开控制输入监视器：

```
rosrun acado_mpc plot_control_input.py
```

你应该能看到监视器显示着两个控制输入话题上的消息，此时由于没有消息发布所以是静止的。

配置mpc的权重参数：

```
roslaunch acado_mpc set_weight.launch
```

然后直接`ctrl+c`取消，此时自定义的权重参数已经传入。

紧接着在这个终端里打开mpc控制节点：

```
rosrun acado_mpc mpc_gt_node
```

至此你应该能看到JACKAL小车开始移动并进行轨迹跟踪。


## Combine C code with ROS nodes and simulate in Gazebo
