#!/usr/bin/env python
"""
  simple tool for robot path analysis
    viewer.py <filename> [--startIndex=<num>] [<scale> [<tile size>]]
"""

import sys

import os, math, pygame
import zipfile
import io

from pygame.locals import *

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


printPosition = True
printEncoders = True

size = 1200, 660
offset = 150, 150
absOffset = 0, 0 # offset of original (x,y) coordinate system (Robotour stromovka = 1049800.66, 5580126.84 )

sample_color = (255,0,0)
sample_size = 20
track_color = (0,255,0)
track_size = 2
tile_size = 0.5

scale = 1.0

# global timestamps - workaround
g_timestamps = []


def deg(degAngle): return math.pi*degAngle/180.0

def draw(surface, samples):
  for s in samples:
    surface.fill(sample_color, (to_px(s[0]), to_px(s[1]), sample_size, sample_size))

def to_px(x): return int(x*150*scale)

def _scr(x,y): return (to_px(x)+offset[0], size[1]-to_px(y)-offset[1])

def scr(x,y): return _scr(x-absOffset[0], y-absOffset[1])

def scr1( point ) : return scr( point[0], point[1] )
#-------------------------------------------------

def getCombinedPose( pose, sensorPose ):
  x = pose[0] + sensorPose[0] * math.cos( pose[2] ) - sensorPose[1] * math.sin( pose[2] )
  y = pose[1] + sensorPose[0] * math.sin( pose[2] ) + sensorPose[1] * math.cos( pose[2] )
  heading = sensorPose[2] + pose[2]
  return (x, y, heading)


def loadData(filename):
    m = []
    poses_set = []

    pose = (0, 0, 0)
    poses = [pose,]
    scans = []
    image = None
    camdir = None
    compass = None

    laser_id = lookup_stream_id(filename, 'lidar.scan')
    camera_id = lookup_stream_id(filename, 'camera.raw')  # TODO refactor
    try:
        pose_id = lookup_stream_id(filename, 'eduro.pose2d')  # TODO what other source than Eduro?
    except ValueError:
        pose_id = None
    
    with LogReader(filename) as log:
        for timestamp, stream_id, data in log.read_gen():
            if stream_id == laser_id:
                scan = deserialize(data)
                angle_scale = 270/(len(scan)-1)
                scans = []
                for i, s in enumerate(scan):
                    angle = math.radians(135) - math.radians(270) * i/(len(scan)-1)
                    angle = -angle  # now it is up
                    dist = s/1000.0
                    if True:  # math.radians(-90) <= angle <= math.radians(+90):
                        scans.append((getCombinedPose(pose, (0, 0, angle)), dist))
                scans.append((pose, -3))  # "MCL pose" (for draw all without sensors)
                poses_set.append((poses, scans, image, camdir, compass))
                g_timestamps.append(timestamp)
            elif stream_id == camera_id:
                image = deserialize(data)
            elif stream_id == pose_id:
                x, y, heading = deserialize(data)
                pose = (x/1000.0, y/1000.0, math.radians(heading/100.0))
                poses = [pose,]
    return poses_set, m


def loadData_legacy( filename ):
#  geometry = ( ( -0.02, 0.04, deg(90) ), ( -0.02, -0.04, deg(-90) ) )
#  geometry = ( ( -0.09, 0.14, deg(90) ), ( -0.09, -0.14, deg(-90) ) )
#  geometry = ( ( -0.09, 0.14, deg(90) ), ( -0.09, -0.14, deg(-90) ), 
#               ( 0.05, 0.10, deg(0) ), ( 0.05, -0.10, deg(0) ) )
  map = []
  geometry = []
  posesSet = []
  poses = []
  scans = []
  image = None
  camdir = None
  compass = None
  readingMap = False
  for line in open( filename ):
    arr = line.rstrip().split()
    if not line.rstrip():
      continue
    if arr[0] == "Map":
      readingMap = True
    elif arr[0] == "Poses":
      readingMap = False
      if poses:
        posesSet.append( (poses, scans, image, camdir, compass) )
      poses = []
      scans = []
      image = None
      camdir = None
      compass = None
    elif arr[0] == "Compass":
      readingMap = False
      compass = float(arr[1])
    elif arr[0] == "Geometry":
      geometry = []
      tmp = arr[1:]
      while len( tmp ) > 0:
        geometry.append( (float(tmp[0]), float(tmp[1]), float(tmp[2])) )
        tmp = tmp[3:]
    elif arr[0] == "Ranger":
      readingMap = False
      i = 0
      for a in arr[4:]:
        scans.append( ( getCombinedPose( (float(arr[1]), float(arr[2]), float(arr[3])), geometry[i] ), float(a)) )      
        i += 1
    elif arr[0] == "Beacon":
      assert( int(arr[1]) == 1 )
      index = len(arr)>4 and int(arr[4]) or 0
      if len(arr) == 7:
        scans.append( ( ( float(arr[2]), float(arr[3]), 0.0 ), -1.5, [int(c) for c in arr[-3:]]) ) # color param
      else:
        scans.append( ( ( float(arr[2]), float(arr[3]), 0.0 ), -1.0-index/10.0) )      
    elif arr[0] == "Puck": 
      assert( int(arr[1]) == 1 )
      scans.append( ( ( float(arr[2]), float(arr[3]), 0.0 ), -2.0) )      
    elif arr[0] == "RefPose":
      assert int(arr[1]) == 1, arr
      scans.append( ( ( float(arr[2]), float(arr[3]), 0.0 ), -3.0) )      
    elif arr[0] == "Image":
      image = arr[1]
    elif arr[0] == "ImageResult":
      try:
        camdir = float(arr[1])
      except: #it cannot be interpreted as a direction
        pass
    else:
      if readingMap:
        map.append( ((float(arr[0]), float(arr[1])), (float(arr[2]),float(arr[3]))) )
      else:
        poses.append( (float(arr[0]), float(arr[1]), float(arr[2])) )
  posesSet.append( (poses,scans,image,camdir, compass) )
  return posesSet, map


def drawPoses( foreground, poses ):
  d = 0.3
  for (x,y,heading) in poses:
    pygame.draw.circle( foreground, (0,0,255), scr(x,y),5 )
    pygame.draw.line( foreground, (255,255,0), scr(x,y), scr(x+d*math.cos(heading),y+d*math.sin(heading)),5)

def drawCompass( foreground, scanArr ):
  if scanArr[4]:
    x,y,heading = scanArr[0][0]
    heading = scanArr[4]
    d = 0.4
    pygame.draw.line( foreground, (0,0,255), scr(x,y), scr(x+d*math.cos(heading),y+d*math.sin(heading)),5)

def drawScans( foreground, scans, shouldDrawSensors, shouldDrawBeacons ):
  for row in scans:
    color = None
    if len(row) == 2:
      ((x,y,heading),range) = row
    else:
      ((x,y,heading),range, color) = row 
    if range >= 0:
      if range > 0:
        if shouldDrawSensors:
          col = (0,255,0)
          if range > 1.0:
            col = (0,100,0)
          pygame.draw.circle( foreground, col, scr(x+range*math.cos(heading),y+range*math.sin(heading)),3)
          pygame.draw.circle( foreground, (255,0,0), scr(x,y), 4 )
    elif range > -1.99:
      # beacons (not nice)
      if shouldDrawBeacons:
        if color == None:
          pygame.draw.circle( foreground, (255,0,int((-1-range)*255*3.3)), scr(x,y), 6 )
        else:
          pygame.draw.circle( foreground, color, scr(x,y), 10 ) 
    elif range > -2.5:
      # pucks (not nice2)
      pygame.draw.circle( foreground, (255,0,168), scr(x,y), 3 )
    else:
      # ref MCL pose (not nice3)
      pygame.draw.circle( foreground, (255,0,0), scr(x,y), 1 )

def drawMap( foreground, map ):
  for m in map:
    pygame.draw.line( foreground, (0,255,255), scr1(m[0]), scr1(m[1]),5)

def drawImage( foreground, imgFileName, camdir ):
  if imgFileName is not None and imgFileName.startswith(b'\xff\xd8'):
      buf = imgFileName # direct image
      camera = pygame.image.load(io.BytesIO(buf), 'JPG').convert()
      cameraView = pygame.transform.scale( camera, (512, 384) )
      foreground.blit( cameraView, (size[0]-512,0) )
      return

  if imgFileName:
#  imgFileName = 'D:\\md\\hg\\eduro-logs\\100619-rychnov\\pes1\\cam100619_145404_000.jpg'
    if '.zip' in imgFileName:
      zipname, filename = os.path.split(imgFileName)
      print(zipname, filename)
      zf = zipfile.ZipFile(zipname)
      buf = zf.read(filename)
      camera = pygame.image.load(io.BytesIO(buf), 'JPG').convert()
    else:
      camera = pygame.image.load( imgFileName ).convert()
#    cameraView = pygame.transform.scale( camera, (320, 240) )
    cameraView = pygame.transform.scale( camera, (512, 384) )
#    cameraView = pygame.transform.flip( cameraView, False, True )
    if camdir is not None:
      color = (0xFF, 0x00, 0x00)
      start_pos = (160, 240)
      length = 80
      end_pos = (160 - length * math.sin(camdir), 240 - length * math.cos(camdir))
      width = 2
      pygame.draw.line(cameraView, color, start_pos, end_pos, width)
#    foreground.blit( cameraView, (size[0]-320,0) )
    foreground.blit( cameraView, (size[0]-512,0) )

def drawTiles( background ):
  background.fill((35, 35, 35))
  tile_size_px = to_px(tile_size)
  for x in range(0, size[0] + tile_size_px, tile_size_px):
    for y in range(0, size[1] + tile_size_px, tile_size_px):
      if (x // tile_size_px) % 2 == (y // tile_size_px) % 2:
        background.fill((235, 235, 235), Rect(x, max(0, size[1] - y), tile_size_px, tile_size_px + min(0, size[1] - y)))
  # draw scale
  pygame.draw.line( background, (255,0,0), (20,size[1]-20), (20+to_px(1.0),size[1]-20),3)
  pygame.draw.line( background, (255,0,0), (20,size[1]-10), (20,size[1]-30),1)
  pygame.draw.line( background, (255,0,0), (20+to_px(1.0),size[1]-10), (20+to_px(1.0),size[1]-30),1)


def main( filename, scale = 1.0, startIndex = None, posesScanSet=None ):
  
  # load pygame
  pygame.init()
  os.environ['SDL_VIDEO_CENTERED'] = '1'
  #screen = pygame.display.set_mode(size, NOFRAME, 0)
  screen = pygame.display.set_mode(size)

  # create backgroud
  background = pygame.Surface(screen.get_size())
  drawTiles(background)

  # create foreground
  foreground = pygame.Surface(screen.get_size())
  foreground.set_colorkey((0,0,0))
  index = 0

  # display everything
  screen.blit(background, (0, 0))
  screen.blit(foreground, (0, 0))
  pygame.display.flip()

  pygame.event.set_blocked(MOUSEMOTION) # keep our queue cleaner
  pygame.key.set_repeat ( 200, 20 )  

  shouldDrawTiles = True
  shouldDrawMap = True
  shouldDrawSensors = False
  shouldDrawBeacons = False
  shouldRefreshNow = False

  if filename is None:
    assert posesScanSet is not None
    map = []
  else:
    posesScanSet, map = loadData( filename )
  drawPoses( foreground, posesScanSet[0][0] )
  drawScans( foreground, posesScanSet[0][1], shouldDrawSensors, shouldDrawBeacons )

  screen.blit(background, (0, 0)) 
  screen.blit(foreground, (0, 0))
  pygame.display.set_caption("Viewer init ...")
  pygame.display.flip() 

  global absOffset
  if startIndex != None:
    index = startIndex
    if index < len(posesScanSet):
      # move robot into center
      pose = posesScanSet[index][0][0]
      absOffset = pose[:2]
  else:
    # skip first non-moving part
    index = 0
    for s in posesScanSet:
      pose = s[0][0]
      index += 1
      if math.fabs(pose[0])+math.fabs(pose[1]) > 1.0:
        absOffset = pose[:2]
        break
    if index >= len(posesScanSet):
      index = len(posesScanSet) - 1
  print(index, len(posesScanSet), absOffset)

  imgFileName = None
  lastImgFileName = None
  while 1:
    if isinstance(imgFileName, bytes):
      t = filename  # 'raw JPEG data'
    elif imgFileName:
      t = str(imgFileName)+' ***'
      lastImgFileName = imgFileName
    else:
      t = str(lastImgFileName)

    timestamp = g_timestamps[index]
    pygame.display.set_caption(str(timestamp) + " Index: %d, sensors %s, %s" % (index, shouldDrawSensors and "on" or "off", t) )
    shouldRefreshNow = False
    event = pygame.event.wait()
    if event.type == QUIT: return
    if event.type == KEYDOWN:
      if event.key in (K_ESCAPE,K_q): return
      if event.key == K_p:
        if index + 1 < len( posesScanSet ):
          index += 1
      if event.key == K_o:
        if index + 10 < len( posesScanSet ):
          index += 10
      if event.key == K_i:
        if index > 0:
          index -= 1
      if event.key == K_0:
        index = 0
      if event.key == K_9:
        index = len( posesScanSet ) - 1
      if event.key == K_m:
        shouldDrawMap = not shouldDrawMap;        
      if event.key == K_s:
        shouldDrawSensors = not shouldDrawSensors;        
      if event.key == K_b:
        shouldDrawBeacons = not shouldDrawBeacons;        
      if event.key == K_RIGHT:
        globals()['offset'] = (offset[0]+150, offset[1])
        shouldRefreshNow = True
      if event.key == K_LEFT:
        globals()['offset'] = (offset[0]-150, offset[1])
        shouldRefreshNow = True
      if event.key == K_UP:
        globals()['offset'] = (offset[0], offset[1]+150)
        shouldRefreshNow = True
      if event.key == K_DOWN:
        globals()['offset'] = (offset[0], offset[1]-150)
        shouldRefreshNow = True
      if event.key == K_PLUS or event.key == K_KP_PLUS or event.key == 61:
        globals()['scale'] *= 2.0
        globals()['tile_size'] /= 2.0
        shouldDrawTiles = shouldDrawMap = shouldDrawSensors = shouldDrawBeacons = True
        shouldRefreshNow = True
      if event.key == K_MINUS or event.key == K_KP_MINUS:
        globals()['scale'] /= 2.0
        globals()['tile_size'] *= 2.0
        shouldDrawTiles = shouldDrawMap = shouldDrawSensors = shouldDrawBeacons = True
        shouldRefreshNow = True

      def has_image(k):
        return bool(posesScanSet[k][2])

#      if True: #hack has_image(index) or shouldRefreshNow:
      if has_image(index) or shouldRefreshNow:
        foreground.fill( (0,0,0) )

      refreshed_since = index

      if shouldRefreshNow:
        while refreshed_since > 0:
          if has_image(refreshed_since):
            break
          refreshed_since -= 1

      for i in range(refreshed_since, index + 1):
        drawPoses( foreground, posesScanSet[i][0] )
        drawCompass( foreground, posesScanSet[i] )
        drawScans( foreground, posesScanSet[i][1], shouldDrawSensors, shouldDrawBeacons )
        drawImage( foreground, posesScanSet[i][2], posesScanSet[i][3] )
        imgFileName = posesScanSet[i][2]

      if event.key == K_a:
        foreground.fill( (0,0,0) )
        for (p,s,i,camdir,comp) in posesScanSet:
          drawScans( foreground, s, shouldDrawSensors, shouldDrawBeacons )

    if shouldDrawTiles:
      drawTiles(background)

    if shouldDrawMap:
      drawMap( foreground, map )
    screen.blit(background, (0, 0)) 
    screen.blit(foreground, (0, 0))
    pygame.display.flip() 


def usage():
  print(__doc__)

if __name__ == "__main__": 
  if len(sys.argv) < 2:
    usage()
    sys.exit(2)
  scale = 1.0

  args = sys.argv[:]
  startIndex = None
  if len(args) > 2 and args[2].startswith("--startIndex"):
    startIndex = int(args[2].split('=')[1])
    args = args[:2]+args[3:]

  if len(args) > 2:
    scale = float( args[2] )
    if len(args) > 3:
      tile_size = float( args[3] )
  main(args[1], scale=scale, startIndex=startIndex)

# vim: expandtab sw=4 ts=4 

