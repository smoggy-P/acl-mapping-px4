# Sliding Mapper #

Blue is unknown space, and orange is occupied-known space:

**UAV**               |  **Ground Robot**          | 
:-------------------------:|:-------------------------:|
[![IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments](./imgs/uav_sim.gif)](https://www.youtube.com/watch?v=fkkkgomkX10 "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments")      |  [![IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments](./imgs/gr_sim.gif)](https://youtu.be/L13k44-krcc "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments") |  

If you find the code of this mapper helpful, please consider citing these papers:

**Efficient Trajectory Planning for High Speed Flight in Unknown Environments** (ICRA 2019) ([pdf](https://ieeexplore.ieee.org/abstract/document/8793930), [video](https://www.youtube.com/watch?v=Wic0-xyC_i8))
```bibtex
@inproceedings{ryll2019efficient,
  title={Efficient Trajectory Planning for High Speed Flight in Unknown Environments},
  author={Ryll, Markus and Ware, John and Carter, John and Roy, Nick},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={732--738},
  year={2019},
  organization={IEEE}
}
```

**FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments** (IROS 2019) ([conference paper](https://arxiv.org/abs/1903.03558), [journal paper](https://arxiv.org/abs/2001.04420), [video](https://www.youtube.com/watch?v=gwV0YRs5IWs))

```bibtex
@inproceedings{tordesillas2019faster,
  title={{FASTER}: Fast and Safe Trajectory Planner for Flights in Unknown Environments},
  author={Tordesillas, Jesus and Lopez, Brett T and How, Jonathan P},
  booktitle={2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2019},
  organization={IEEE}
}

```

**Real-Time Planning with Multi-Fidelity Models for Agile Flights in Unknown Environments** (ICRA 2019) ([pdf](https://arxiv.org/abs/1810.01035), [video](https://www.youtube.com/watch?v=E4V2_B8x-UI))
```bibtex
@article{tordesillas2018real,
  title={Real-Time Planning with Multi-Fidelity Models for Agile Flights in Unknown Environments},
  author={Tordesillas, Jesus and Lopez, Brett T and Carter, John and Ware, John and How, Jonathan P},
  journal={arXiv preprint arXiv:1810.01035},
  year={2018}
}
```

## Commands:
`roslaunch global_mapper_ros global_mapper_node.launch`

## Credits:
This mapper was developed mainly by John Ware and John Carter (Robust Robotics Group, MIT), so all the credit goes to them. 

Jesus Tordesillas (ACL-MIT) did some minor modifications afterwards (in the branch `dev`)
