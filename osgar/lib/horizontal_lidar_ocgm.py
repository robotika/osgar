import numpy as np
from osgar.lib.ocgm import Ocgm
#from scipy.ndimage.interpolation import map_coordinates as mp
import scipy.ndimage.interpolation as ndi
import cv2
import math
from scipy.sparse import csr_matrix
import pdb

#OCGM parameters
OCGM_CELLS_COUNT = 400
OCGM_CELLS_PER_METER = 18
LIDAR_ANGLE = 0.0 #angle of lidar to the horizont 
LIDAR_HEIGHT = 0.63 #m
LIDAR_Y_SHIFT = 0.2 #m
IGNORE_NEAR_OBSTACLES_DIST = 0.2 #m
MIN_LIDAR_ANGLE = 40 #degrees
MAX_LIDAR_ANGLE = 230 #degrees


class HorizontalLidarOcgm(Ocgm):
    
    def __init__(self, cells, res, p_d, p_nd, decay,pose,cols,rows):
        super(HorizontalLidarOcgm, self).__init__(cells, res, p_d, p_nd, decay,pose,cols,rows)

        # --- tours and polar map definition ---
        self._dt_map = 'f4'
        self._map_prior = 0.0
        self._map =  self._map_prior * np.ones([int(self._map_shape[0]),int(self._map_shape[1])], dtype=self._dt_map)
        
        #--- min/max values for plot ---
        self._v_max = 1.0
        self._v_min = 0.0
        self.lastTimestamp = None

    def update(self, scan, timestamp, pose):
        RESOLUTION_STEP = 270 / len(scan) * math.pi / 180
        angle = - 135 * RESOLUTION_STEP    
        scanConverted = []
        previousScan = 0
        for i in range (1,len(scan)):
            if scan[i] == 0:
                currentScan = 0
            else:                        
                currentScan = scan[i]/1000
                
            #OCGM: don't calculate cartesian coordinates here. Sparse grid needs polar...
            y = currentScan * math.cos(LIDAR_ANGLE) * math.cos(angle) + LIDAR_Y_SHIFT 
            x = -currentScan * math.sin(angle)
            newPoint = [x,y,math.degrees(i * RESOLUTION_STEP),math.sqrt(x*x + y*y)]
            if MIN_LIDAR_ANGLE  <= math.degrees(i * RESOLUTION_STEP) <= MAX_LIDAR_ANGLE: 
                scanConverted.append(newPoint) 
                pointX = newPoint[0]*math.cos(pose[2]) - newPoint[1]*math.sin(pose[2])
                pointY = newPoint[0]*math.sin(pose[2]) + newPoint[1]*math.cos(pose[2])
                relativePositionX = pose[0] 
                relativePositionY = pose[1] 
                currentPointPosition = [relativePositionX,relativePositionY,0]
                
            angle += RESOLUTION_STEP
            previousScan = currentScan
        
        csrData = []
        csrRow = []
        csrCol = []
        index = 0
        nextPoint = scanConverted[0]
        
        for i in range(0,len(scanConverted)-1):                                               
            point = scanConverted[i]
            
            if point[0] != 0 and point[3] != 0 and point[3] - LIDAR_Y_SHIFT > IGNORE_NEAR_OBSTACLES_DIST:
                nextPoint = scanConverted[i+1]
                if  nextPoint[3] == 0:
                    continue
                csrData.append(1)
                csrRow.append(MAX_LIDAR_ANGLE- point[2])
                if point[3]*OCGM_CELLS_PER_METER <= 200:
                    csrCol.append(point[3]*OCGM_CELLS_PER_METER) #resolution 10cm
                else:
                    csrCol.append(200)
                
        csr = csr_matrix((np.array(csrData, dtype=np.float64), (csrRow, csrCol)), shape=(270, OCGM_CELLS_COUNT))
        csrImage = np.array(csr.toarray()*255, dtype=np.uint8)
        #cv2.imshow('Lidar CSR Image', csrImage)
        if self.lastTimestamp == None:
            self.lastTimestamp = timestamp
            
        timeDelta = timestamp - self.lastTimestamp
        self.lastTimestamp = timestamp
        vehicleData, mapData = self._update(pose,0, 0, csr, timeDelta.total_seconds())
        mapVisualization = self._logOdds2Image(mapData)
        shape = mapVisualization.shape
        cv2.circle(mapVisualization, (int(shape[0]/2),int(shape[1]/2)), 2, 255,-1)
        cv2.imshow('Map Lidar', mapVisualization)
        
        #rotatedMap = ndi.rotate(mapData,-pose[2]*180/math.pi)
        #cv2.circle(rotatedMap, (int(shape[0]/2),int(shape[1]/2)), 2, 255,-1)
        #cv2.imshow('Rotated Map Lidar', self._logOdds2Image(rotatedMap))

    def _correct(self, csr):
        self._polar_map[:] = 0.0

        self._csr_m = csr  # sparse matrix
        self._csr_m.data *= self._p_d
        
        nonzero_rows = self._csr_m.indptr[1:] - self._csr_m.indptr[:-1] != 0
        for p in range(self._polar_map.shape[0]):
            if nonzero_rows[p]:
                self._polar_map[p, :self._csr_m.indices[self._csr_m.indptr[p]] - 1] = \
                  self._ism[:self._csr_m.indices[self._csr_m.indptr[p]] - 1]
                    ##
                self._polar_map[p, self._csr_m.indices[self._csr_m.indptr[p]]] = \
                    self._csr_m.data[self._csr_m.indptr[p]]
            else:
                self._polar_map[p, :] = self._ism
        coords = self._calc_torus_coords(csr)
        #cv2.imshow('Polar map Lidar', self._polar_map*255)
        
        newUpdate = ndi.map_coordinates(self._polar_map, coords,
                        order=0, mode='constant', cval=0.0, prefilter=False).reshape(self._map_shape[:2])
        #cv2.imshow('newUpdate Lidar', newUpdate*255)
        
        # ---  apply occupancy map  ---
        self._map += newUpdate
        
        
    def _logOdds2Image(self,logodds):
        retiled = np.minimum(np.maximum(logodds, self._min_log), -self._min_log)
        _exp = np.exp(retiled)
        _exp = _exp / (1.0 + _exp)
        _exp = np.array(np.exp(_exp)*255,dtype=np.uint8)
        return _exp
        
        
    
    def _calc_torus_coords(self, scan):
        res_phi = math.pi/180 #1 degree resolution
        
        # ---  sensor and vehicle offsets  ---
        x_off = self._xy_frac[0]
        y_off = self._xy_frac[1]

        # ---  transform coordinates  ---
        torus_g_x = np.zeros(self._g.shape)
        torus_g_y = np.zeros(self._g.shape)
        torus_g_x[:int(self._edge_pt[0])] = self._g[int(self._g.shape[0] - self._edge_pt[0]):]
        torus_g_x[int(self._edge_pt[0]):] = self._g[:int(self._g.shape[0] - self._edge_pt[0])]

        torus_g_y[int(self._g.shape[0] - self._edge_pt[1]):] = self._g[:int(self._edge_pt[1])]
        torus_g_y[:int(self._g.shape[0] - self._edge_pt[1])] = self._g[int(self._edge_pt[1]):]

        self._mg_x, self._mg_y = np.meshgrid(torus_g_x, torus_g_y)
        self._mg_r_flat = (np.sqrt((self._mg_x - x_off) ** 2 + (self._mg_y - y_off) ** 2)).ravel()
        self._mg_phi_flat = - np.arctan2(self._mg_y - y_off, self._mg_x - x_off).ravel()
        self._mg_phi_flat += self._phi + 175/180*np.pi #compensation of the sensor position

        # ---  stack coordinates  ---
        return np.vstack(((-self._mg_phi_flat % (2 * np.pi)) / res_phi,
                          self._mg_r_flat / self._res_r))


    def _evolve(self):
        self._map *= self._decay**(self._delta_t/0.040)



    def _extract(self):
        # retile torus
        retiled_map = self._map_prior * np.ones(self._map_shape, dtype=self._dt_map)
        # lower right plaquette
        retiled_map[:self._edge_pt[1], :self._map_shape[1] - self._edge_pt[0]] = \
            self._map[self._map_shape[0] - self._edge_pt[1]:, self._edge_pt[0]:]
        # upper left plaquette
        retiled_map[:self._edge_pt[1], self._map_shape[1] - self._edge_pt[0]:] = \
            self._map[self._map_shape[0] - self._edge_pt[1]:, :self._edge_pt[0]]
        # upper right plaquette
        retiled_map[self._edge_pt[1]:, :self._map_shape[1] - self._edge_pt[0]] = \
            self._map[:self._map_shape[0] - self._edge_pt[1], self._edge_pt[0]:]
        # upper left plaquette
        retiled_map[self._edge_pt[1]:, self._map_shape[1] - self._edge_pt[0]:] = \
            self._map[:self._map_shape[0] - self._edge_pt[1], :self._edge_pt[0]]

        #retiled_map = np.minimum(np.maximum(retiled_map, 0.0), 1.0)
        
        return retiled_map