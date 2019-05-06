import numpy as np
from scipy.sparse import csr_matrix
import pdb
import math

class Ocgm(object):
    
    def __init__(self,cells, res, p_d, p_nd, decay,pose,cols,rows):
        # --- map definition ---
        self._cells = cells
        self._res = res
        self._xy_max = self._cells * self._res

        self._map_shape = np.array([int(2 * self._cells), int(2 * self._cells)])

        self._bin_edges_xy = np.linspace(-self._xy_max, self._xy_max, 2 * self._cells + 1)
        self._bin_edges = [self._bin_edges_xy, self._bin_edges_xy]

        self._edge_pt = np.array([0, 0])
        self._polar_map = np.zeros((cols, rows), dtype='f4')
        self._csr_m = csr_matrix

        # --- mesh
        self._g = 0.5 * (self._bin_edges[0][:-1] + self._bin_edges[0][1:])
        self._mg_x, self._mg_y = np.meshgrid(self._g, self._g)

        self._mg_r_flat = (np.sqrt(self._mg_x ** 2 + self._mg_y ** 2)).ravel()
        self._mg_phi_flat = - np.arctan2(self._mg_y, self._mg_x).ravel()

        # ---  unconditioned free space  ---
        # the values for the radius are fixed, while the angle ranged depend on
        # which sensor provides the currently processed data
        self._extent_r = (0, self._xy_max)
        self._ranges = np.linspace(self._extent_r[0], self._extent_r[1], self._polar_map.shape[1])
        self._res_r = float(self._extent_r[1] - self._extent_r[0]) / (self._polar_map.shape[1] - 1)
        
        # --- probability values
        self._min_log = -70
        self._log_odd_p_d = np.log(p_d) - np.log1p(- p_d)
        self._p_d = p_d

        self._p_nd = p_nd
        self._log_odd_p_nd = np.log(p_nd) - np.log1p(- p_nd)

                                                 
        #--- inverse sensor model ---
        #my
        #self._ism = np.zeros((rows), dtype='f4')
        #self._ism = 1.0 - 0.5 * (1.0 - np.tanh((self._ranges - 10.0) / 40.0))
        self._ism = 1.0 - 0.5 * (1.0 - np.tanh((self._ranges - 10.0) / 40.0))
        self._ism *= 0.5 - self._p_nd
        self._ism += self._p_nd
        self._ism = np.log(self._ism) - np.log1p(- self._ism)
        
        
        # ---  car position and orientation for prediction ---
        self._tmp_dx = np.zeros(2)
        self._xy_frac = np.zeros(2)

        self._phi = pose[2]
        self._sin_phi = np.cos(self._phi)
        self._cos_phi = np.sin(self._phi)

        self._dt_vehicle = np.dtype([('xy', 'f4', (2,)), ('phi', 'f4')])
        self._vehicle_data = np.zeros(1, dtype=self._dt_vehicle)

        # ---  shift values [cells] ---
        self._dx_cells = 0
        self._dy_cells = 0


        # --- time parameters and thresholds ---
        self._epsilon = 1e-9
        self._delta_t = 0.0
        self._time_offset = 0.0
        self._us_to_sec = 1e-6

        self._decay = decay

        self.lastPose = pose

    def _update(self, pose, velocity_mps, yaw_rate_radps, csr, time_delta):
        # ---  limit steps  ---
        vx = -velocity_mps
        omega = -yaw_rate_radps
        omega = 0

        #self._phi = np.pi * 0.75-pose.getHeading()
        self._phi = -pose[2]
        self._sin_phi = np.cos(self._phi)
        self._cos_phi = np.sin(self._phi)

        self._delta_t = time_delta
        
        self._predict(vx, omega,pose)
        self._correct(csr)

        self._vehicle_data['xy'] = self._xy_frac
        self._vehicle_data['phi'] = self._phi
        map_data = self._extract()

        return self._vehicle_data, map_data



    def _evolve(self):
        raise NotImplementedError


    def _predict(self, vx, omega,pose):
        self._evolve()

        self._calculate_shift(vx, omega,pose)

        self._shift_map()


    def _calculate_shift(self, vx, omega,pose):
        _pos_shift_total = np.array([pose[1] - self.lastPose[1], pose[0] - self.lastPose[0]])  + self._xy_frac
        self.lastPose = pose
        _pos_shift_int = np.round(_pos_shift_total / self._res).astype(int)
        self._xy_frac = _pos_shift_total - self._res * _pos_shift_int.astype(float)
        self._dx_cells = _pos_shift_int[0]
        self._dy_cells = _pos_shift_int[1]



    def _shift_map(self):
        new_edge_pt = self._edge_pt + np.asarray([self._dx_cells, self._dy_cells])
        new_edge_pt_mod = np.mod(new_edge_pt, [self._map_shape[1], self._map_shape[0]])
        # shift in x-direction
        if self._dx_cells < 0:
            if new_edge_pt[0] < 0:  # underflow
                self._map[:, :int(self._edge_pt[0])] = self._map_prior
                self._map[:, int(new_edge_pt_mod[0]):] = self._map_prior
            else:
                self._map[:, new_edge_pt_mod[0]: self._edge_pt[0]] = self._map_prior
        else:  # positive shift distance
            if new_edge_pt[0] >= self._map.shape[1]:  # overflow
                self._map[:, int(self._edge_pt[0]):] = self._map_prior
                self._map[:, :int(new_edge_pt_mod[0])] = self._map_prior
            else:  # vanilla
                self._map[:, int(self._edge_pt[0]): int(new_edge_pt_mod[0])] = self._map_prior

        # shift in y-direction
        if self._dy_cells < 0:  # negative shift distance
            if new_edge_pt[1] < 0:  # underflow
                self._map[:int(self._map_shape[0] - new_edge_pt_mod[1])] = self._map_prior
                self._map[int(self._map_shape[0] - self._edge_pt[1]):] = self._map_prior
            else:
                self._map[int(self._map_shape[0] - self._edge_pt[1]): int(self._map_shape[0] - new_edge_pt_mod[1])] = self._map_prior
        else:  # positive shift distance
            if new_edge_pt[1] > self._map_shape[0]:  # overflow  torus_map.shape[0] - new_edge_pt_mod[1] < 0
                self._map[int(self._map_shape[0] - new_edge_pt_mod[1]):] = self._map_prior
                self._map[:int(self._map_shape[0] - self._edge_pt[1])] = self._map_prior
            else:  # vanilla
                self._map[int(self._map_shape[0] - new_edge_pt_mod[1]): int(self._map_shape[0] - self._edge_pt[1])] = self._map_prior

        self._edge_pt = new_edge_pt_mod


    def _correct(self, scan):
        raise NotImplementedError


    def _extract(self):
        raise NotImplementedError