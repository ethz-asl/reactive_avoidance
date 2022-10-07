#!/usr/bin/env python3
import subprocess
from multiprocessing import Pool
import os
import numpy as np
import glob
import re

NUMPROCESSES = 1

class ExpRun():
    default_dict = {
        # GENERAL
        "obstacles" : [200],
        "n_runs" : 10,
        "seed" : 0,
        "data_path" : "../eval/data/",
        "world_path" : "../eval/world/",
        "world_load_path": "../eval/data/world/custom/tsdf/voxblox_ground_truth_demo.tsdf",
        "stats_only" : 0,
        "policy_type" : 1, # 1 = RMP raycaster, 0 = ESDF following
        "planner_type": 0,
        "world_type": 1,

        #"""RMP"""

        "metric" : 1,
        "N_sqrt": 32,
        "trunc_dist": 1.0,
        "r": 2.4,      #2.4
        "dt": 0.06, #0.06
        "v_rep_damp": 1.2, # 1.2
        "terminate_upon_goal_reached": 1,
    }

    def run_single(self, d):
        curdir = os.path.dirname(__file__)
        bin = os.path.join("/home/mpantic/ws/rmp/devel/lib/rmpcpp_planner", "rmpcpp_planner_runtest")
        datapath = os.path.join(curdir, d["data_path"])
        worldpath = os.path.join(curdir, d["world_path"])
        newd = d.copy()
        newd["data_path"] = datapath
        newd["world_path"] = worldpath
        os.makedirs(datapath, exist_ok=True)
        os.makedirs(worldpath, exist_ok=True)
        cmd = [os.path.normpath(bin)] + list(np.array([[f"--{key}", str(value)] for key, value in newd.items()]).flatten())
        print(cmd)
        with open(datapath + "conf.txt", "w") as f:
            f.write(" ".join(cmd))
        proc = subprocess.Popen(cmd)
        proc.wait()


    def run_obstacle_experiment(self, datapath, worldpath):
        if NUMPROCESSES > 1:
            pool = Pool(NUMPROCESSES)
        for obstacles in self.default_dict["obstacles"]:
            d = self.default_dict.copy()
            d["data_path"] = f"{datapath}obstacles{str(obstacles)}/"
            d["world_path"] = f"{worldpath}obstacles{str(obstacles)}/"
            d["obstacles"] = str(obstacles)
            # Seed gets incremented for every run, so now every run has a unique seed. We do max with 100 to make sure that if we do a lower number of runs than
            # 100, the maps still have the same seed. If you do more than 100 runs, you are going to get conflicting plots so don't do that.
            d["seed"] = obstacles * max(100, d["n_runs"])
            if(obstacles in [50, 100, 150, 200]):
                d["stats_only"] = 0
            if NUMPROCESSES > 1:
                pool.apply_async(ExpRun.run_single, args=(self, d))
            else:
                self.run_single(d)
        if NUMPROCESSES > 1:
            pool.close()
            pool.join()




def run_multiple_experiments(experiments, prepend, custom=False):
    for experiment in experiments.keys():
        for value in experiments[experiment]:
            datapath= f"../data/stats/{prepend}{experiment}/{value}/"
            worldpath = "../data/world/"
            exp = ExpRun()
            copy = exp.default_dict.copy()
            if experiment in exp.default_dict:  # Possible to also run things that are not in the parameter list
                exp.default_dict[experiment] = value

            exp.run_obstacle_experiment(datapath, worldpath)
            exp.default_dict = copy.copy() # Restore default dict

if __name__ == "__main__":

    """Run the main experiments"""

    prepend = "testrun-"
    experiments = { # RMP
        "N_sqrt": [32]
    }
    run_multiple_experiments(experiments, prepend)
