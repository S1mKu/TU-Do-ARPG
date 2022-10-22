# hands


## Add python dependencies to rosdep

- create dependencies yaml
  
      sudo nano /etc/ros/rosdep/sources.list.d/artus-dependencies.yaml

- write rules into it

      python3-do-mpc:
        ubuntu:
          pip:
            packages: [do-mpc]

- create list file

      sudo nano /etc/ros/rosdep/sources.list.d/10-artus-dependencies.list

- copy path into it

      yaml file:///etc/ros/rosdep/sources.list.d/artus-dependencies.yaml

- update rosdep

      rosdep update

## generate solver
solver needs to be generated before running mpc nodes

        cd path_to_package/src/time_optimal_mpc/utils/
        python3 mpc_generation_script.py