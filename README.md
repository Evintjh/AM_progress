<NTU URECA 22/23>
# AM_audio_monitoring_ws

For acoustic feature extraction visualisation on Plotjuggler: Launch acoustic_monitoring.launch
- Run ROS command: roslaunch acoustic_feature_extraction acoustic_monitoring.launch

  
For PID (C++) on PlotJuggler: Launch laser_sim.launch file 
- Run ROS command: roslaunch laam_laser_control laser_sim.launch
- Integral Control in PID (C++):
  - Bi-linear (Tustin, Trapezoidal) technique to transform integral control to z-domain / discrete domain.
  - Transforming to discrete domain allows better performance compared to Euler (Forward) and Backward rule, which is commonly used in time-domain integral control.
  - https://www.semanticscholar.org/paper/Bilinear-Discrete-PID%C3%97(n-2)-stage-PD-cascade-for-Smerpitak-Ukakimaparn/4c3b9d24d9c8a3af1189585f226fad599fe828c8


For PID (Python) with Qt GUI: Launch control.launch file
- Run ROS command: roslaunch laam_laser_control control.launch
