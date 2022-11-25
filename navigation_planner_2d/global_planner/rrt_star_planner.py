"""
    RRT-star Planner as a global planner
    Kohei Honda, 2022
"""

from __future__ import annotations

from typing import Tuple, List
import numpy as np
import matplotlib.pyplot as plt

from navigation_planner_2d.global_planner.global_planner_base import GlobalPlannerBase
from navigation_simulator_2d.utils import MapHandler, ParameterHandler
from navigation_planner_2d.global_planner.rrt_base import Tree, RRTBase

class RRTStarPlanner(RRTBase):
    def __init__(self, params: ParameterHandler) -> None:
        super().__init__(params)
        
    def make_plan(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> Tuple:
        
        # initialize tree
        self._reset()
        self._add_vertex(start_pos)
        
        is_goal = False
        for _ in range(self._max_iter):
            # random sample pos
            x = self._rng.uniform(self._area_limits[0], self._area_limits[1])
            y = self._rng.uniform(self._area_limits[2], self._area_limits[3])
            sample_pos = np.array([x, y])
            
            # get nearest vertex
            nearest_vertex, nearest_index = self._get_nearest_vertex(sample_pos)
            
            # get new vertex
            new_vertex = self._get_new_vertex(nearest_vertex, sample_pos)
            
            # check validity
            is_valid = self._check_validity(nearest_vertex, new_vertex)
                        
            if is_valid:
                # find candidate vertices to be rewired
                candidate_indices = self._find_near_vertices(new_vertex)
                
                # choose parent vertex
                # TODO: From here implement RRT-star
                parent_vertex, parent_index = self._choose_parent_vertex(new_vertex, candidate_indices)

                # connect to the tree
                self._connect_tree(parent_index, new_vertex)
                
                # rewrite tree
                # self._rewire(new_vertex, candidate_indices)
                
                 # check goal reachability
                if self._check_goal_reachable(goal_pos, new_vertex):
                    is_goal = True
                    break
        
        if is_goal:
            path = self._reconstruct_path(start_pos, goal_pos)
        else:
            # return best effort path
            nearest_vertex_to_goal, _ = self._get_nearest_vertex(goal_pos)
            path = self._reconstruct_path(start_pos, nearest_vertex_to_goal)
        
        path_in_costmap = self._cost_map.pose_array2index_array(path)
    
        path_batch = np.expand_dims(path, axis=0)
            
        return path_in_costmap, path_batch
            
            
            
    def _find_near_vertices(self, new_vertex: np.ndarray) -> List[int]:
        """
        Find near vertices
        params: new_vertex: np.ndarray
        return: List[int]: near indices of vertices
        """
        
        num_vertices = self._tree.vertices.shape[0]
        
        if num_vertices == 0:
            raise ValueError("No vertices in the tree")
        
        if num_vertices == 1:
            return [0]
        
        # radius is adaptive to the number of vertices
        weight = 50.0
        radius = weight * np.sqrt(np.log(num_vertices) / num_vertices)
        
        # find vertices within radius from new vertex
        dists = np.linalg.norm(self._tree.vertices - new_vertex, axis=1)
        candidate_indices = np.where(dists < radius)[0]
        
        return candidate_indices
    
    
    def _choose_parent_vertex(self, new_vertex: np.ndarray, candidate_indices: List[int]) -> Tuple[np.ndarray, int]:
        pass
    
    def _rewire(self, new_vertex: np.ndarray, candidate_indices: List[int]) -> None:
        # TODO: Tree クラスに実装
        pass